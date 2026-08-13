#ifndef __MEM_ROWOP_TRACE_PLAYER_HH__
#define __MEM_ROWOP_TRACE_PLAYER_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "mem/request.hh"
#include "params/RowOpTracePlayer.hh"
#include "sim/eventq.hh"

/**
 * RowOpTracePlayer
 *
 * Reads a CIMTRACE binary (from the Cinnamon compiler), expands each
 * MULI / ADDI / ROW_COPY record into the corresponding ROWAP / ROWAAP /
 * ROWCOPY packet sequence, and replays them against a DRAMCtrl.
 *
 * Packets are sent one at a time (send → wait WriteResp → send next) so
 * the DRAM timing model runs without a CPU.
 *
 * Address layout (example: RoRaBaCoCh + DDR4_2400_x64, banks/rank=16, ranks=2):
 *   ROW_SIZE          = 8192 B (macro from request.hh)
 *   banks_per_channel = 32     (banksPerRank * ranksPerChannel)
 *   ALIGNMENT         = 262144 B
 *
 *   channel    = global_bank / banks_per_channel
 *   local_bank = global_bank % banks_per_channel
 *   slot_addr(slot, global_bank) =
 *       base_addr + channel * channel_size
 *                 + (slot * banks_per_channel + local_bank) * row_stride
 *
 *   row_stride = device_rowbuffer_size from the DRAM timing config.
 *   For DDR4 row_stride == ROW_SIZE (8192); for HBM2/HBM3 row_stride == 1024.
 *   Using the DRAM row buffer size as the stride ensures DRAMCtrl's bank/row
 *   decode assigns each (slot, local_bank) pair to the correct DRAM bank and
 *   keeps all slots within a single 512-row subarray.
 *
 *   The trace addresses up to 128 global banks (128-bit bank mask), split into
 *   channels = 128 / banks_per_channel non-interleaved DRAMCtrls:
 *     DDR4 (banks_per_channel=32) :  4 channels
 *     HBM3 (banks_per_channel=16) :  8 channels
 *     HBM2 (banks_per_channel=8)  : 16 channels
 *
 *   Slots 0-17 : ambit control rows (T0..C_1, matching init_ambit())
 *   Slots 18+  : data pools (lhs, rhs, out, partial, tmp, carry)
 */
class RowOpTracePlayer : public MemObject
{
  private:

    // ------------------------------------------------------------------ //
    // Control-row slot indices (must match init_ambit() in mimdram.h)
    // ------------------------------------------------------------------ //
    enum : int {
        SLOT_T0 = 0, SLOT_T1, SLOT_T2, SLOT_T3,
        SLOT_DCC0, SLOT_DCC0N, SLOT_DCC1, SLOT_DCC1N,
        SLOT_DCC0N_T0, SLOT_DCC1N_T1,
        SLOT_T2_T3, SLOT_T0_T3, SLOT_T0_T1_T2, SLOT_T1_T2_T3,
        SLOT_DCC0_T1_T2, SLOT_DCC1_T0_T3,
        SLOT_C_0, SLOT_C_1,
        SLOT_DATA_BASE = 18
    };

    // ROW_SIZE = 8192 is already defined as a macro in request.hh.
    // Legacy constant from the obsolete 32-bank format; the live bank count is
    // derived per-trace (up to 128 for the current format) as
    // banksPerChannel * num_channels, so this member is unused.  Kept only for
    // reference.  Banks are partitioned across channels:
    //   channel    = global_bank / banksPerChannel
    //   local_bank = global_bank % banksPerChannel
    static const int TOTAL_BANKS = 32;

    // Rows per subarray (must match DRAMCtrl::rowsPerSubarray, mimdram.h
    // ROWS_PER_SUBARRAY, and trace_player.py).  slotAddr() maps slot->row
    // linearly, so adding/subtracting ROWS_PER_SUBARRAY to a slot moves it
    // into a neighbouring subarray at the same within-subarray offset — this
    // is how the FCDRAM backend addresses its cross-subarray reference rows.
    //
    // FCDRAM layout (3 subarrays per bank): compute rows live in the MIDDLE
    // subarray (slotOffset = ROWS_PER_SUBARRAY shifts every slot there);
    // the two neighbouring subarrays (slot ± ROWS_PER_SUBARRAY) hold the
    // reference rows.  Open-bitline sharing means each neighbour's sense
    // amps cover only HALF of a compute row's columns (FCDRAM paper, §5
    // footnote 6), so every cross-subarray gate is issued once per side.
    static const int ROWS_PER_SUBARRAY = 512;

    // ------------------------------------------------------------------ //
    // Row-op expansion backend.  Each value lowers the same CIMTRACE
    // schedule onto a different PuD substrate.  Extension points for a new
    // backend: add an enumerator here, a name in parseBackend(), and a
    // per-backend expandAdd*/expandMul* pair dispatched by expandAdd/expandMul.
    // ------------------------------------------------------------------ //
    enum Backend {
        BK_SIMDRAM,   // Ambit AAP/AP        (94_simdram_schedule_runner.c)
        BK_FCDRAM,    // COTS cross-subarray (96_fcdram_schedule_runner.c)
        BK_PRADA      // SRA: TRA + N + 5RA  (95_prada_schedule_runner.c)
    };
    static Backend parseBackend(const std::string& name);

    // ------------------------------------------------------------------ //
    // One pending row-op (slot indices; address computed at send time)
    // ------------------------------------------------------------------ //
    struct PendingOp {
        Request::RowOp op;
        int dest_slot;
        int dest_bank;
        int src1_slot;
        int src1_bank;
        int src2_slot; // unused for current Ambit ops
        int src2_bank;
        int64_t start; // scheduler start-time of the originating trace record
    };

    // ------------------------------------------------------------------ //
    // Master port
    // ------------------------------------------------------------------ //
    class TraceMasterPort : public MasterPort {
      public:
        TraceMasterPort(const std::string& n, RowOpTracePlayer& p)
            : MasterPort(n, &p), player(p) {}
      protected:
        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
        void recvTimingSnoopReq(PacketPtr)  {}
        void recvFunctionalSnoop(PacketPtr) {}
        Tick recvAtomicSnoop(PacketPtr)     { return 0; }
      private:
        RowOpTracePlayer& player;
    };

    TraceMasterPort port;

    // ------------------------------------------------------------------ //
    // Per-channel master port: carries a channel id so retries and responses
    // route to that channel's independent state.  One per channel, each with
    // its own gem5 retry slot, so a stall on channel c does not block issues to
    // the other channels.  Used only when perChannel is true.
    // ------------------------------------------------------------------ //
    class ChanMasterPort : public MasterPort {
      public:
        ChanMasterPort(const std::string& n, RowOpTracePlayer& p, int ch)
            : MasterPort(n, &p), player(p), chan(ch) {}
      protected:
        bool recvTimingResp(PacketPtr pkt)
            { return player.recvTimingRespChan(chan, pkt); }
        void recvReqRetry() { player.recvReqRetryChan(chan); }
        void recvTimingSnoopReq(PacketPtr)  {}
        void recvFunctionalSnoop(PacketPtr) {}
        Tick recvAtomicSnoop(PacketPtr)     { return 0; }
      private:
        RowOpTracePlayer& player;
        int chan;
    };

    std::vector<ChanMasterPort*> chanPorts;

    // ------------------------------------------------------------------ //
    // Parameters / IDs
    // ------------------------------------------------------------------ //
    const bool        perChannel;      // use the per-channel issue engine
    const std::string traceFile;
    const Addr        baseAddr;
    const int         banksPerChannel; // banks per DRAMCtrl instance
    const Addr        channelSize;     // byte stride between channel base addrs
    const Addr        rowStride;       // DRAM row buffer size (address stride per slot)
    const Backend     backend;         // row-op expansion backend (see enum Backend)
    const bool        singleBank;      // single-bank opt: 1 rep bank/channel (see .py)
    // FCDRAM: compute/data rows live in the MIDDLE of 3 subarrays so both
    // cross-subarray neighbours (slot ± ROWS_PER_SUBARRAY) are addressable;
    // slotAddr() adds this to every slot.  0 for single-subarray backends.
    const int         slotOffset;
    MasterID          masterID;

    // ------------------------------------------------------------------ //
    // Runtime state
    // ------------------------------------------------------------------ //
    std::vector<PendingOp> pendingOps;

    // Multiple-outstanding issue engine, grouped by the schedule's start-time.
    // All row-ops of one start-group are issued concurrently (bounded only by
    // the memory's own back-pressure), so independent banks/channels overlap
    // exactly as the scheduler assumes.  A dependency barrier between groups
    // (issue group G+1 only after all of group G completes) preserves the
    // schedule's ordering.  This makes the measured runtime reflect bank/
    // channel parallelism instead of the total per-bank packet count.
    std::vector<size_t> groupEnds;   // cumulative end index of each start-group
    size_t    curGroup;              // index of the group currently in flight
    size_t    issueIdx;              // next pendingOps index to issue
    size_t    completedCount;        // total responses received
    PacketPtr retryPkt;

    // ------------------------------------------------------------------ //
    // Per-channel engine state (used only when perChannel is true).  The
    // start-group barrier is kept: only the CURRENT start-group's ops are
    // partitioned across channels, each channel draining its own sub-stream
    // (in order) through its own port + retry slot.  When the whole group has
    // completed, the next group is partitioned and pumped.
    // ------------------------------------------------------------------ //
    std::vector<std::vector<size_t>> chanQ;      // current group's ops per channel
    std::vector<size_t>    chanCursor;           // next slot in chanQ[c]
    std::vector<PacketPtr> chanRetry;            // retry pkt per channel (null=none)

    // Row-op makespan: tick the first row-op is accepted by the memory and
    // tick the last row-op completes.  Their difference is the wall-clock span
    // of the row-op phase (the benchmark runtime), excluding harness startup.
    bool      firstOpSeen;
    Tick      firstOpTick;
    Tick      lastOpTick;

    // Data-pool slot offsets (set from the trace's max widths)
    int lhsBase, rhsBase, outBase, partialBase, tmpBase, carryBase;

    // ------------------------------------------------------------------ //
    // Port callbacks (called by TraceMasterPort)
    // ------------------------------------------------------------------ //
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();

    // ------------------------------------------------------------------ //
    // Trace loading + expansion
    // ------------------------------------------------------------------ //
    void loadTrace();

    // Backend-dispatching expanders: pick the per-backend implementation.
    void expandAdd(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandSub(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandMul(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    // ROW_COPY (inter-bank bus copy) is backend-independent — shared verbatim.
    void expandRowCopy(int bw, int src_bank, int dst_bank);
    // The five ops below are implemented for simdram and prada.  fcdram
    // panics: its gates are cross-subarray with open-bitline half-row
    // coverage and a destructive MAJ3, so every one of these sequences would
    // need its own reference-row re-initialisation analysis, and guessing it
    // would break the exact compiler/simulator op-count agreement the whole
    // evaluation rests on.
    //
    // Bit-serial ReLU: out[i] = lhs[i] AND ~sign, for every bit row i.
    void expandRelu(int bw, const std::vector<int>& banks);
    // Widening add: N-bit + N-bit -> (N+1)-bit, keeping the carry-out row.
    void expandAddWide(int bw, const std::vector<int>& banks);

    // Bitwise XNOR, the binary-neural-network multiply.
    void expandXnor(int bw, const std::vector<int>& banks);

    // BitWeaving/V BETWEEN range scan over a bw-bit bit-sliced column.
    void expandRangeScan(int bw, const std::vector<int>& banks);

    // Bit-serial min/max (compare then select).  One implementation serves
    // both -- they differ only in which operand each mux branch selects.
    void expandMinMax(int bw, const std::vector<int>& banks, bool isMax);

    // --- SIMDRAM backend (Ambit AAP/AP) ------------------------------- //
    void expandAddSimdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandSubSimdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandMulSimdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);

    // Ambit primitive emitters
    void emitAAP (int dst_slot, int src_slot, int bank);
    void emitAP  (int dst_slot,               int bank);
    void emitCopy(int dst_slot, int dst_bank, int src_slot, int src_bank);

    // Cross-channel row-copy halves (one full-row data-bus stream per side)
    void emitRdStream(int slot, int bank);
    void emitWrStream(int slot, int bank);

    // Ambit multi-bank composite emitters
    void emitRowAnd(int lhs_slot, int rhs_slot, int out_slot,
                    const std::vector<int>& banks);
    void emitRowAdd(int lhs_slot, int rhs_slot, int out_slot,
                    int cin_slot, int cout_slot,
                    const std::vector<int>& banks);

    // --- FCDRAM backend (COTS DDR4 cross-subarray gates) -------------- //
    void expandAddFcdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandMulFcdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);

    // COTS primitive emitters.  ROWCLONE is an intra-subarray copy; the *_XSUB
    // gates are cross-subarray and address their reference rows at the com
    // row's mirrors in BOTH neighbouring subarrays (com_slot ± RPS).  Due to
    // open-bitline half-row coverage each gate is one APA per side (2 packets)
    // and the AND/OR emitters also re-initialise the reference pair per side
    // (ROWCLONE of the VDD/GND threshold row + FRAC of the VDD/2 row), since
    // the previous gate overwrote it with the NAND/NOR byproduct (paper §6.1.3).
    void emitClone  (int dst_slot, int src_slot, const std::vector<int>& banks);
    void emitFrac   (int dst_slot,               const std::vector<int>& banks);
    void emitAndXsub(int com_slot,               const std::vector<int>& banks);
    void emitOrXsub (int com_slot,               const std::vector<int>& banks);
    void emitNotXsub(int dst_slot,               const std::vector<int>& banks);
    // COTS 3-input majority via simultaneous multi-row activation
    // (FracDRAM/PULSAR): com = MAJ(com, s1, s2), intra-subarray.  The
    // activation group includes a VDD/2 helper row (SLOT_DCC1N) that must be
    // re-FRAC'd before every MAJ3, and restoration writes the majority back
    // into ALL activated rows, destroying s1/s2 — callers stage operands that
    // must survive into scratch copies.
    void emitMaj3   (int com_slot, int s1_slot, int s2_slot,
                     const std::vector<int>& banks);

    // COTS multi-bank composite emitters (mirror the C helpers in 96_...)
    void emitRowAndFc(int lhs_slot, int rhs_slot, int out_slot,
                      const std::vector<int>& banks);
    void emitRowAddFc(int lhs_slot, int rhs_slot, int out_slot,
                      int cin_slot, int cout_slot,
                      const std::vector<int>& banks);

    // --- PRADA backend (PRADA, ICCAD'24: Sequential Row Activation) ---- //
    // Same bit-serial shift-add schedule as SIMDRAM, but the bit-row AND and
    // full-adder are built from PRADA's distinctive row-ops instead of Ambit
    // AAP chains: TRA (3-row majority, ROWAAAP), the single-command NOT (N,
    // ROWANAP; no DCC) and 5RA (5-row majority, ROWAAAAAP).  No new gem5
    // primitive is needed -- these ops already exist with faithful timing.
    void expandAddPrada(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandMulPrada(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandSubPrada     (int lhs_bw, int rhs_bw,
                             const std::vector<int>& banks);
    void expandAddWidePrada (int bw, const std::vector<int>& banks);
    void expandReluPrada    (int bw, const std::vector<int>& banks);
    void expandXnorPrada    (int bw, const std::vector<int>& banks);
    void expandRangeScanPrada(int bw, const std::vector<int>& banks);
    void expandMinMaxPrada  (int bw, const std::vector<int>& banks, bool isMax);
    // PRADA OR: out = MAJ(1, lhs, rhs), the dual of emitRowAndPrada.
    void emitRowOrPrada(int lhs_slot, int rhs_slot, int out_slot,
                        const std::vector<int>& banks);

    // PRADA primitive emitters (per single bank).  ROWAAAP = A A As P (a
    // 3-activation sequence: TRA majority OR a two-destination copy -- same
    // timing either way).  ROWANAP = As N A P (sense, invert, copy).
    // ROWAAAAAP = A A A A As P (5-activation 5-row majority).
    void emitAAAP  (int dst_slot, int src1_slot, int src2_slot, int bank);
    void emitANAP  (int dst_slot, int src_slot,                 int bank);
    void emitAAAAAP(int dst_slot, int src1_slot, int src2_slot, int bank);

    // PRADA multi-bank composite emitters (mirror the C helpers in 95_...)
    void emitRowAndPrada(int lhs_slot, int rhs_slot, int out_slot,
                         const std::vector<int>& banks);
    void emitRowAddPrada(int lhs_slot, int rhs_slot, int out_slot,
                         int cin_slot, int cout_slot,
                         const std::vector<int>& banks);

    // Address + packet helpers
    Addr      slotAddr(int slot, int bank) const;
    PacketPtr makeRowOpPacket(Request::RowOp op,
                               Addr dest, Addr src1, Addr src2);
    // Build the DRAM packet for pendingOps[idx] (shared by both engines).
    PacketPtr makeOpPacket(size_t idx);
    // Channel a pending op targets: dest bank's owning DRAMCtrl.
    int       opChannel(size_t idx) const
        { return pendingOps[idx].dest_bank / banksPerChannel; }

    // ------------------------------------------------------------------ //
    // Send-event (declare sendNextOp BEFORE the EventWrapper)
    // ------------------------------------------------------------------ //
    void sendNextOp();
    EventWrapper<RowOpTracePlayer, &RowOpTracePlayer::sendNextOp> sendEvent;

    // ------------------------------------------------------------------ //
    // Per-channel engine (used only when perChannel is true)
    // ------------------------------------------------------------------ //
    void partitionGroup(size_t g);   // fill chanQ[] with group g's ops
    void pumpChannel(size_t c);      // issue channel c until back-pressure
    void chanPumpAll();              // the per-channel send step (all channels)
    bool recvTimingRespChan(int ch, PacketPtr pkt);
    void recvReqRetryChan(int ch);
    EventWrapper<RowOpTracePlayer, &RowOpTracePlayer::chanPumpAll> chanSendEvent;

  public:

    RowOpTracePlayer(const RowOpTracePlayerParams* p);

    BaseMasterPort& getMasterPort(const std::string& if_name,
                                  PortID idx = InvalidPortID);

    void startup();
    void regStats();

    // ------------------------------------------------------------------ //
    // Statistics
    // ------------------------------------------------------------------ //
    Stats::Scalar numPacketsSent;
    Stats::Scalar numRetries;
    // Wall-clock span of the row-op phase, in ticks (last completion - first
    // issue).  This is the benchmark runtime metric.
    Stats::Scalar rowOpMakespan;
};

#endif // __MEM_ROWOP_TRACE_PLAYER_HH__
