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
 * Address layout for RoRaBaCoCh + DDR4_2400_x64 (banks/rank=16, ranks=2):
 *   ROW_SIZE     = 8192 B (macro from request.hh)
 *   ROWS_PER_VEC = 32     (banksPerRank * ranksPerChannel)
 *   ALIGNMENT    = 262144 B
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
 *   DDR4 (banks_per_channel=32, 1 channel) : reduces to the original formula.
 *   HBM2 (banks_per_channel=8,  4 channels): global banks 0-7 → ch0, 8-15 → ch1, …
 *   HBM3 (banks_per_channel=16, 2 channels): global banks 0-15 → ch0, 16-31 → ch1.
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
    // TOTAL_BANKS = 32 matches mimdram.h BANK_COUNT(16) * RANK_COUNT(2).
    // Banks are partitioned across channels:
    //   channel   = global_bank / banksPerChannel
    //   local_bank = global_bank % banksPerChannel
    static const int TOTAL_BANKS = 32;

    // Rows per subarray (must match DRAMCtrl::rowsPerSubarray, mimdram.h
    // ROWS_PER_SUBARRAY, and trace_player.py).  slotAddr() maps slot->row
    // linearly, so adding ROWS_PER_SUBARRAY to a slot moves it into the
    // neighbouring subarray at the same within-subarray offset — this is how
    // the FCDRAM backend places a cross-subarray reference row (see below).
    static const int ROWS_PER_SUBARRAY = 512;

    // ------------------------------------------------------------------ //
    // Row-op expansion backend.  Each value lowers the same CIMTRACE
    // schedule onto a different PuD substrate.  Extension points for a new
    // backend: add an enumerator here, a name in parseBackend(), and a
    // per-backend expandAdd*/expandMul* pair dispatched by expandAdd/expandMul.
    // ------------------------------------------------------------------ //
    enum Backend {
        BK_SIMDRAM,   // Ambit AAP/AP        (94_simdram_schedule_runner_hbm.c)
        BK_FCDRAM     // COTS cross-subarray (96_fcdram_schedule_runner_hbm.c)
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
    // Parameters / IDs
    // ------------------------------------------------------------------ //
    const std::string traceFile;
    const Addr        baseAddr;
    const int         banksPerChannel; // banks per DRAMCtrl instance
    const Addr        channelSize;     // byte stride between channel base addrs
    const Addr        rowStride;       // DRAM row buffer size (address stride per slot)
    const Backend     backend;         // row-op expansion backend (see enum Backend)
    MasterID          masterID;

    // ------------------------------------------------------------------ //
    // Runtime state
    // ------------------------------------------------------------------ //
    std::vector<PendingOp> pendingOps;
    size_t    currentOp;
    bool      waitingResp;
    PacketPtr retryPkt;

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
    void expandMul(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    // ROW_COPY (inter-bank bus copy) is backend-independent — shared verbatim.
    void expandRowCopy(int bw, int src_bank, int dst_bank);

    // --- SIMDRAM backend (Ambit AAP/AP) ------------------------------- //
    void expandAddSimdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);
    void expandMulSimdram(int lhs_bw, int rhs_bw, const std::vector<int>& banks);

    // Ambit primitive emitters
    void emitAAP (int dst_slot, int src_slot, int bank);
    void emitAP  (int dst_slot,               int bank);
    void emitCopy(int dst_slot, int dst_bank, int src_slot, int src_bank);

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
    // gates are cross-subarray and read their reference from the neighbouring
    // subarray at the com row's mirror offset (com_slot + ROWS_PER_SUBARRAY),
    // matching the A-in-subarray-S / B-in-subarray-S+1 layout of the COTS
    // microworkloads (e.g. MIMDRAM_ext/microworkloads/31_and_cots.c).
    void emitClone  (int dst_slot, int src_slot, const std::vector<int>& banks);
    void emitAndXsub(int com_slot,               const std::vector<int>& banks);
    void emitOrXsub (int com_slot,               const std::vector<int>& banks);
    void emitNotXsub(int dst_slot,               const std::vector<int>& banks);

    // COTS multi-bank composite emitters (mirror the C helpers in 96_...)
    void emitRowAndFc(int lhs_slot, int rhs_slot, int out_slot,
                      const std::vector<int>& banks);
    void emitRowAddFc(int lhs_slot, int rhs_slot, int out_slot,
                      int cin_slot, int cout_slot,
                      const std::vector<int>& banks);

    // Address + packet helpers
    Addr      slotAddr(int slot, int bank) const;
    PacketPtr makeRowOpPacket(Request::RowOp op,
                               Addr dest, Addr src1, Addr src2);

    // ------------------------------------------------------------------ //
    // Send-event (declare sendNextOp BEFORE the EventWrapper)
    // ------------------------------------------------------------------ //
    void sendNextOp();
    EventWrapper<RowOpTracePlayer, &RowOpTracePlayer::sendNextOp> sendEvent;

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
};

#endif // __MEM_ROWOP_TRACE_PLAYER_HH__
