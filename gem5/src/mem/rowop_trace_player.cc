#include "mem/rowop_trace_player.hh"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "base/trace.hh"
#include "debug/RowOpTracePlayer.hh"
#include "mem/packet.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

// ---------------------------------------------------------------------------
// On-disk trace structures.
//
// Two binary formats exist, distinguished by TraceHeader::record_size:
//
//   record_size == 32  ->  32-bank format  (93_simdram_schedule_runner.c)
//                          banks field: uint32_t (bits 0..31)
//
//   record_size == 48  ->  128-bank format (94_simdram_schedule_runner_hbm.c)
//                          banks field: uint64_t[2] (bits 0..127)
//
// After reading, both are normalised into NormRecord for uniform processing.
// ---------------------------------------------------------------------------
namespace {

struct TraceHeader {
    char     magic[8];
    uint32_t version;
    uint32_t record_size;
    uint64_t num_records;
    int64_t  last_end_time;
};
static_assert(sizeof(TraceHeader) == 32, "TraceHeader size mismatch");

// 32-bank on-disk record (record_size == 32)
struct TraceRecord32 {
    int64_t  start;
    int64_t  end;
    uint32_t banks;
    uint16_t lhs_bw;
    uint16_t rhs_bw;
    uint8_t  kind;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  pad[5];
};
static_assert(sizeof(TraceRecord32) == 32, "TraceRecord32 size mismatch");

// 128-bank on-disk record (record_size == 48)
struct TraceRecord128 {
    int64_t  start;
    int64_t  end;
    uint64_t banks[2];   // banks[b>>6] bit (b&63) = bank b active
    uint16_t lhs_bw;
    uint16_t rhs_bw;
    uint8_t  kind;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  pad[9];
};
static_assert(sizeof(TraceRecord128) == 48, "TraceRecord128 size mismatch");

// Normalised record used internally after reading either format.
struct NormRecord {
    int64_t  start;
    uint64_t banks[2];
    int      lhs_bw;
    int      rhs_bw;
    uint8_t  kind;
    uint8_t  src;
    uint8_t  dst;
};

enum OpKind : uint8_t { OP_MULI = 0, OP_ADDI = 1, OP_ROW_COPY = 2 };

} // anonymous namespace

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

// Out-of-class definition for the ODR-used static const member (passed by
// reference to the variadic panic()/inform() helpers).
const int RowOpTracePlayer::ROWS_PER_SUBARRAY;

// ---------------------------------------------------------------------------
// Backend name -> enum.  Central extension point: register a new backend's
// name here (and add its expandAdd*/expandMul* implementations below).
// ---------------------------------------------------------------------------
RowOpTracePlayer::Backend
RowOpTracePlayer::parseBackend(const std::string& name)
{
    if (name == "simdram") return BK_SIMDRAM;
    if (name == "fcdram")  return BK_FCDRAM;
    fatal("RowOpTracePlayer: unknown backend '%s' (expected 'simdram' or "
          "'fcdram')", name.c_str());
    return BK_SIMDRAM; // unreachable; silences -Wreturn-type
}

RowOpTracePlayer::RowOpTracePlayer(const RowOpTracePlayerParams* p)
    : MemObject(p),
      port("port", *this),
      traceFile(p->trace_file),
      baseAddr(p->base_addr),
      banksPerChannel(p->banks_per_channel),
      channelSize(p->channel_size),
      rowStride(p->row_stride),
      backend(parseBackend(p->backend)),
      masterID(p->system->getMasterId(name())),
      curGroup(0), issueIdx(0), completedCount(0),
      retryPkt(nullptr),
      firstOpSeen(false), firstOpTick(0), lastOpTick(0),
      lhsBase(SLOT_DATA_BASE), rhsBase(0), outBase(0),
      partialBase(0), tmpBase(0), carryBase(0),
      sendEvent(this)
{
}

// ---------------------------------------------------------------------------
// Port
// ---------------------------------------------------------------------------

BaseMasterPort&
RowOpTracePlayer::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    return MemObject::getMasterPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Stats
// ---------------------------------------------------------------------------

void
RowOpTracePlayer::regStats()
{
    MemObject::regStats();

    numPacketsSent
        .name(name() + ".numPacketsSent")
        .desc("Total row-op packets sent to DRAM");

    numRetries
        .name(name() + ".numRetries")
        .desc("Number of send retries due to back-pressure");

    rowOpMakespan
        .name(name() + ".rowOpMakespan")
        .desc("Row-op phase makespan in ticks (last completion - first issue)");
}

// ---------------------------------------------------------------------------
// Address helper
//
//   global_bank ∈ [0, TOTAL_BANKS)   (always 32, matches mimdram.h)
//   channel    = global_bank / banksPerChannel
//   local_bank = global_bank % banksPerChannel
//
//   slot_addr = base + channel * channelSize
//                    + (slot * banksPerChannel + local_bank) * ROW_SIZE
//
//   For DDR4 (banksPerChannel=32, channelSize unused):
//     channel=0, local_bank=global_bank → base + (slot*32+bank)*ROW_SIZE  ✓
//   For HBM2 (banksPerChannel=8, 4 channels):
//     bank 0-7 → ch0 at base; bank 8-15 → ch1 at base+channelSize; …
//   For HBM3 (banksPerChannel=16, 2 channels):
//     bank 0-15 → ch0 at base; bank 16-31 → ch1 at base+channelSize.
// ---------------------------------------------------------------------------

Addr
RowOpTracePlayer::slotAddr(int slot, int global_bank) const
{
    int channel    = global_bank / banksPerChannel;
    int local_bank = global_bank % banksPerChannel;
    return baseAddr
         + (Addr)channel * channelSize
         + ((Addr)slot * banksPerChannel + local_bank) * rowStride;
}

// ---------------------------------------------------------------------------
// Packet factory
// ---------------------------------------------------------------------------

PacketPtr
RowOpTracePlayer::makeRowOpPacket(Request::RowOp op,
                                   Addr dest, Addr src1, Addr src2)
{
    Request* req = new Request(dest,
                               sizeof(Request::RowOpPayload),
                               Request::UNCACHEABLE | Request::ROWOP,
                               masterID);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);

    // Allocate as uint8_t[] so Packet::deleteData()'s `delete [] data` is
    // well-defined (dataDynamic<T> stores the pointer as uint8_t* internally
    // and the destructor always uses array-delete).
    uint8_t* raw = new uint8_t[sizeof(Request::RowOpPayload)];
    Request::RowOpPayload payload = {op, dest, src1, src2};
    memcpy(raw, &payload, sizeof(payload));
    pkt->dataDynamic(raw);

    return pkt;
}

// ---------------------------------------------------------------------------
// Primitive emitters
// ---------------------------------------------------------------------------

void RowOpTracePlayer::emitAAP(int dst_slot, int src_slot, int bank)
{
    pendingOps.push_back({Request::ROWAAP,
                          dst_slot, bank,
                          src_slot, bank,
                          0,        bank});
}

void RowOpTracePlayer::emitAP(int dst_slot, int bank)
{
    pendingOps.push_back({Request::ROWAP,
                          dst_slot, bank,
                          0, bank,
                          0, bank});
}

void RowOpTracePlayer::emitCopy(int dst_slot, int dst_bank,
                                 int src_slot, int src_bank)
{
    pendingOps.push_back({Request::ROWCOPY,
                          dst_slot, dst_bank,
                          src_slot, src_bank,
                          0, 0});
}

// ---------------------------------------------------------------------------
// Composite emitters (match C helpers in 93_simdram_schedule_runner.c)
// ---------------------------------------------------------------------------

// execute_row_and: T0←lhs, T1←rhs, T2←C_0, out←T0∧T1∧T2
void
RowOpTracePlayer::emitRowAnd(int lhs_slot, int rhs_slot, int out_slot,
                               const std::vector<int>& banks)
{
    for (int b : banks) emitAAP(SLOT_T0,       lhs_slot,    b);
    for (int b : banks) emitAAP(SLOT_T1,       rhs_slot,    b);
    for (int b : banks) emitAAP(SLOT_T2,       SLOT_C_0,    b);
    for (int b : banks) emitAAP(out_slot,      SLOT_T0_T1_T2, b);
}

// execute_row_add: full-adder, cin→cout stored in caller-supplied slots
void
RowOpTracePlayer::emitRowAdd(int lhs_slot, int rhs_slot, int out_slot,
                              int cin_slot, int cout_slot,
                              const std::vector<int>& banks)
{
    for (int b : banks) emitAAP(SLOT_DCC1,     cin_slot,       b);
    for (int b : banks) emitAAP(SLOT_T0_T1_T2, SLOT_DCC1,      b);
    for (int b : banks) emitAAP(SLOT_T2_T3,    lhs_slot,       b);
    for (int b : banks) emitAAP(SLOT_DCC1,     rhs_slot,       b);
    for (int b : banks) emitAAP(cout_slot,     SLOT_DCC1_T0_T3, b);
    for (int b : banks) emitAAP(SLOT_T0_T3,    SLOT_DCC1N,     b);
    for (int b : banks) emitAP (SLOT_T0_T1_T2,                  b);
    for (int b : banks) emitAAP(SLOT_T1,       rhs_slot,       b);
    for (int b : banks) emitAAP(out_slot,      SLOT_T1_T2_T3,  b);
}

// ---------------------------------------------------------------------------
// Backend dispatchers.  loadTrace() calls these; each forwards to the
// per-backend implementation.  Add a case here when introducing a backend.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandAdd(int lhs_bw, int rhs_bw,
                             const std::vector<int>& banks)
{
    switch (backend) {
      case BK_SIMDRAM: expandAddSimdram(lhs_bw, rhs_bw, banks); break;
      case BK_FCDRAM:  expandAddFcdram (lhs_bw, rhs_bw, banks); break;
    }
}

void
RowOpTracePlayer::expandMul(int lhs_bw, int rhs_bw,
                             const std::vector<int>& banks)
{
    switch (backend) {
      case BK_SIMDRAM: expandMulSimdram(lhs_bw, rhs_bw, banks); break;
      case BK_FCDRAM:  expandMulFcdram (lhs_bw, rhs_bw, banks); break;
    }
}

// ---------------------------------------------------------------------------
// SIMDRAM execute_add  (mirrors C code in 94_simdram_schedule_runner_hbm.c)
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandAddSimdram(int lhs_bw, int rhs_bw,
                                    const std::vector<int>& banks)
{
    int bw = std::min(lhs_bw, rhs_bw);

    // init carry = 0 (DCC1 ← ~(DCC1 & C_0))
    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_0, b);

    for (int j = 0; j < bw; j++) {
        int lhs_slot = lhsBase + j;
        int rhs_slot = rhsBase + j;
        int out_slot = outBase + j;

        for (int b : banks) emitAAP(SLOT_T0_T1_T2,  SLOT_DCC1,      b);
        for (int b : banks) emitAAP(SLOT_T2_T3,     lhs_slot,        b);
        for (int b : banks) emitAAP(SLOT_DCC1,      rhs_slot,        b);
        for (int b : banks) emitAP (SLOT_DCC1_T0_T3,                  b);
        for (int b : banks) emitAAP(SLOT_T0_T3,     SLOT_DCC1N,      b);
        for (int b : banks) emitAP (SLOT_T0_T1_T2,                    b);
        for (int b : banks) emitAAP(SLOT_T1,        rhs_slot,         b);
        for (int b : banks) emitAAP(out_slot,       SLOT_T1_T2_T3,   b);
    }
}

// ---------------------------------------------------------------------------
// SIMDRAM execute_mul  (mirrors C code in 94_simdram_schedule_runner_hbm.c)
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandMulSimdram(int lhs_bw, int rhs_bw,
                                    const std::vector<int>& banks)
{
    // Phase 1: init out[0] and partial[] from rhs[0]
    emitRowAnd(lhsBase + 0, rhsBase + 0, outBase + 0, banks);

    for (int i = 0; i < lhs_bw - 1; i++)
        emitRowAnd(lhsBase + i + 1, rhsBase + 0, partialBase + i, banks);

    for (int b : banks) emitAAP(carryBase + 1, SLOT_C_0, b);

    // Phase 2: accumulate rhs[1..rhs_bw-2]
    for (int i = 0; i < rhs_bw - 1; i++) {
        emitRowAnd(lhsBase + 0, rhsBase + i, tmpBase + 0, banks);
        emitRowAdd(tmpBase + 0, partialBase + 0, outBase + i,
                   SLOT_C_0,    carryBase + 0,   banks);

        for (int j = 1; j < lhs_bw - 1; j++) {
            emitRowAnd(lhsBase + j, rhsBase + i, tmpBase + 0, banks);
            emitRowAdd(tmpBase + 0, partialBase + j, partialBase + j - 1,
                       carryBase + 0, carryBase + 0, banks);
        }

        emitRowAnd(lhsBase + lhs_bw - 1, rhsBase + i, tmpBase + 0, banks);
        emitRowAdd(tmpBase + 0, carryBase + 0, partialBase + lhs_bw - 2,
                   carryBase + 1, carryBase + 1, banks);
    }

    // Phase 3: final row rhs[rhs_bw-1]
    emitRowAnd(lhsBase + 0, rhsBase + rhs_bw - 1, tmpBase + 0, banks);

    for (int i = 1; i < lhs_bw - 1; i++) {
        emitRowAnd(lhsBase + i, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
        emitRowAdd(tmpBase + 0, partialBase + i - 1, outBase + rhs_bw - 1 + i,
                   carryBase + 0, carryBase + 0, banks);
    }

    emitRowAnd(lhsBase + lhs_bw - 1, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
    emitRowAdd(tmpBase + 0, carryBase + 1, outBase + lhs_bw + rhs_bw - 2,
               carryBase + 0, outBase + lhs_bw + rhs_bw - 1, banks);
}

// ===========================================================================
// FCDRAM backend (COTS DDR4 functionally-complete gates)
//
// Mirrors 96_fcdram_schedule_runner_hbm.c.  ADDI/MULI keep the exact same
// bit-serial shift-add schedule as the SIMDRAM backend; only the bit-row AND
// and full-adder primitives change, now built from ROWCLONE (intra-subarray
// copy), AND_XSUB / OR_XSUB / NOT_XSUB (cross-subarray gates) and MAJ3 (native
// COTS intra-subarray 3-input majority).  The full adder uses MAJ3 for both
// carry and sum instead of reconstructing them from {AND,OR,NOT}.
//
// Cross-subarray gates require the compute (com/dst) row and its reference to
// live in neighbouring subarrays of the same bank.  slotAddr() maps slot->row
// linearly, so a slot's neighbouring-subarray mirror is slot + ROWS_PER_SUBARRAY
// (same within-subarray offset => the APA activates exactly one row per
// subarray, N=1, matching one bit-plane per bit-serial op).  This reproduces
// the A-in-subarray-S / B-in-subarray-S+1 operand layout of the COTS
// microworkloads (MIMDRAM_ext/microworkloads/{00_addition,31_and}_cots.c).
//
// Like the Ambit backend, this player models op *type/count/timing* only, not
// functional results (the abstract-memory AAP/AP handlers are likewise no-ops).
// The *_XSUB emitters therefore read the reference from the com row's own
// mirror; the logical second operand named in the comments/parameters documents
// intent and is not separately addressed.
// ===========================================================================

// ROWCLONE: dst <- src, intra-subarray copy (both slots in subarray 0).
void
RowOpTracePlayer::emitClone(int dst_slot, int src_slot,
                             const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::ROWCLONE,
                              dst_slot, b, src_slot, b, 0, b});
}

// AND_XSUB: com = com AND ref, ref at com's neighbouring-subarray mirror.
void
RowOpTracePlayer::emitAndXsub(int com_slot, const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::AND_XSUB,
                              com_slot, b,
                              com_slot + ROWS_PER_SUBARRAY, b, 0, b});
}

// OR_XSUB: com = com OR ref, ref at com's neighbouring-subarray mirror.
void
RowOpTracePlayer::emitOrXsub(int com_slot, const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::OR_XSUB,
                              com_slot, b,
                              com_slot + ROWS_PER_SUBARRAY, b, 0, b});
}

// NOT_XSUB: dst = ~src, src at dst's neighbouring-subarray mirror.
void
RowOpTracePlayer::emitNotXsub(int dst_slot, const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::NOT_XSUB,
                              dst_slot, b,
                              dst_slot + ROWS_PER_SUBARRAY, b, 0, b});
}

// MAJ3: com = MAJ(com, s1, s2), intra-subarray triple-row activation.  All
// three operands live in subarray 0 (no mirror), so dram_ctrl accepts it as an
// intra-subarray op; timing reuses the charge-sharing MAJ latency (majBank).
void
RowOpTracePlayer::emitMaj3(int com_slot, int s1_slot, int s2_slot,
                            const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::MAJ3,
                              com_slot, b, s1_slot, b, s2_slot, b});
}

// execute_row_and:  out = lhs AND rhs   (ROWCLONE then AND_XSUB)
void
RowOpTracePlayer::emitRowAndFc(int lhs_slot, int rhs_slot, int out_slot,
                                const std::vector<int>& banks)
{
    (void)rhs_slot; // reference operand; addressed via the com-row mirror
    emitClone  (out_slot, lhs_slot, banks);  // out = lhs
    emitAndXsub(out_slot,           banks);  // out = out AND rhs
}

// execute_row_add:  MAJ-based full adder using the native COTS 3-input MAJ.
//   cout = MAJ(a, b, cin)
//   sum  = MAJ(~cout, cin, MAJ(a, b, ~cout))   ; a ^ b ^ cin
// 8 row-ops (4 CLONE + 1 NOT + 3 MAJ3) vs. 16 for the {AND,OR,NOT} expansion.
// Staging rows: B_T3=cout, B_T2=~cout, B_T0=inner=MAJ(a,b,~cout).  This is a
// timing model, so NOT_XSUB reads its neighbouring-subarray mirror (the
// logical operand ~cout in B_T3 documents intent); MAJ3 operands are addressed
// intra-subarray for the same-subarray/count assertions in dram_ctrl.
void
RowOpTracePlayer::emitRowAddFc(int lhs_slot, int rhs_slot, int out_slot,
                                int cin_slot, int cout_slot,
                                const std::vector<int>& banks)
{
    // cout = MAJ(a, b, cin) -> B_T3
    emitClone(SLOT_T3, lhs_slot, banks);                 // B_T3 = a
    emitMaj3 (SLOT_T3, rhs_slot, cin_slot, banks);        // B_T3 = MAJ(a,b,cin)=cout

    // ~cout -> B_T2
    emitNotXsub(SLOT_T2, banks);                          // B_T2 = ~cout

    // inner = MAJ(a, b, ~cout) -> B_T0
    emitClone(SLOT_T0, lhs_slot, banks);                 // B_T0 = a
    emitMaj3 (SLOT_T0, rhs_slot, SLOT_T2, banks);         // B_T0 = MAJ(a,b,~cout)

    // sum = MAJ(~cout, cin, inner) -> out
    emitClone(out_slot, SLOT_T2, banks);                 // out = ~cout
    emitMaj3 (out_slot, cin_slot, SLOT_T0, banks);        // out = a^b^cin = sum

    // commit carry last (aliasing cout==cin is safe: cin read before this)
    emitClone(cout_slot, SLOT_T3, banks);                // cout = carry-out
}

// FCDRAM execute_add — ripple-carry adder (one full adder per bit).
void
RowOpTracePlayer::expandAddFcdram(int lhs_bw, int rhs_bw,
                                   const std::vector<int>& banks)
{
    int bw = std::min(lhs_bw, rhs_bw);

    // carry := 0 (B_DCC1 is the in-place ripple carry row)
    emitClone(SLOT_DCC1, SLOT_C_0, banks);

    for (int j = 0; j < bw; j++)
        emitRowAddFc(lhsBase + j, rhsBase + j, outBase + j,
                     SLOT_DCC1, SLOT_DCC1, banks);
}

// FCDRAM execute_mul — shift-add multiplier (schedule identical to SIMDRAM).
void
RowOpTracePlayer::expandMulFcdram(int lhs_bw, int rhs_bw,
                                   const std::vector<int>& banks)
{
    // Phase 1: init out[0] and partial[] from rhs[0]
    emitRowAndFc(lhsBase + 0, rhsBase + 0, outBase + 0, banks);

    for (int i = 0; i < lhs_bw - 1; i++)
        emitRowAndFc(lhsBase + i + 1, rhsBase + 0, partialBase + i, banks);

    emitClone(carryBase + 1, SLOT_C_0, banks);   // carry[1] := 0

    // Phase 2: accumulate rhs[1..rhs_bw-2]
    for (int i = 0; i < rhs_bw - 1; i++) {
        emitRowAndFc(lhsBase + 0, rhsBase + i, tmpBase + 0, banks);
        emitRowAddFc(tmpBase + 0, partialBase + 0, outBase + i,
                     SLOT_C_0,    carryBase + 0,   banks);

        for (int j = 1; j < lhs_bw - 1; j++) {
            emitRowAndFc(lhsBase + j, rhsBase + i, tmpBase + 0, banks);
            emitRowAddFc(tmpBase + 0, partialBase + j, partialBase + j - 1,
                         carryBase + 0, carryBase + 0, banks);
        }

        emitRowAndFc(lhsBase + lhs_bw - 1, rhsBase + i, tmpBase + 0, banks);
        emitRowAddFc(tmpBase + 0, carryBase + 0, partialBase + lhs_bw - 2,
                     carryBase + 1, carryBase + 1, banks);
    }

    // Phase 3: final row rhs[rhs_bw-1]
    emitRowAndFc(lhsBase + 0, rhsBase + rhs_bw - 1, tmpBase + 0, banks);

    for (int i = 1; i < lhs_bw - 1; i++) {
        emitRowAndFc(lhsBase + i, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
        emitRowAddFc(tmpBase + 0, partialBase + i - 1, outBase + rhs_bw - 1 + i,
                     carryBase + 0, carryBase + 0, banks);
    }

    emitRowAndFc(lhsBase + lhs_bw - 1, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
    emitRowAddFc(tmpBase + 0, carryBase + 1, outBase + lhs_bw + rhs_bw - 2,
                 carryBase + 0, outBase + lhs_bw + rhs_bw - 1, banks);
}

// ---------------------------------------------------------------------------
// execute_row_copy_batch (single task: src_bank -> dst_bank for bw slots)
//
// Same-channel copy: emit a ROWCOPY packet handled entirely within one
// DRAMCtrl/AbstractMemory instance.
//
// Cross-channel copy: the ROWCOPY primitive in abstract_mem.cc accesses
// both dest and src1 relative to a single channel's pmemAddr base.  When
// the two banks are in different channels the src1 offset would go out of
// bounds and crash.  Model the inter-channel transfer as two ROWAP
// activations instead -- one on the source side (read) and one on the
// destination side (write).  This is a timing approximation but keeps
// all address arithmetic within a single channel's backing store.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandRowCopy(int bw, int src_bank, int dst_bank)
{
    int src_channel = src_bank / banksPerChannel;
    int dst_channel = dst_bank / banksPerChannel;

    if (src_channel == dst_channel) {
        for (int j = 0; j < bw; j++)
            emitCopy(lhsBase + j, dst_bank, outBase + j, src_bank);
    } else {
        // Cross-channel: approximate inter-channel data transfer as
        // ROWAP on src (row read) + ROWAP on dst (row write).
        for (int j = 0; j < bw; j++) {
            emitAP(outBase  + j, src_bank);  // src channel: row activation
            emitAP(lhsBase  + j, dst_bank);  // dst channel: row activation
        }
    }
}

// ---------------------------------------------------------------------------
// loadTrace: two-pass over the binary file
//   Pass 1 – determine max bit-widths → assign slot offsets
//   Pass 2 – expand every record into PendingOps
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::loadTrace()
{
    FILE* f = fopen(traceFile.c_str(), "rb");
    if (!f)
        panic("RowOpTracePlayer: cannot open trace file '%s'", traceFile.c_str());

    TraceHeader hdr;
    if (fread(&hdr, sizeof(hdr), 1, f) != 1 ||
        memcmp(hdr.magic, "CIMTRACE", 8) != 0 ||
        hdr.version != 1)
        panic("RowOpTracePlayer: invalid trace header in '%s'", traceFile.c_str());

    // Detect format from record_size; derive total bank count.
    int total_banks;
    if (hdr.record_size == sizeof(TraceRecord32)) {
        total_banks = 32;
    } else if (hdr.record_size == sizeof(TraceRecord128)) {
        total_banks = 128;
    } else {
        panic("RowOpTracePlayer: unrecognised record_size=%u in '%s' "
              "(expected 32 or 48)", hdr.record_size, traceFile.c_str());
    }

    // --- Read and normalise all records in one pass ---
    std::vector<NormRecord> records(hdr.num_records);
    int max_lhs = 0, max_rhs = 0, max_out = 0, max_partial = 0;

    for (uint64_t i = 0; i < hdr.num_records; i++) {
        NormRecord& nr = records[i];
        if (total_banks == 32) {
            TraceRecord32 raw;
            if (fread(&raw, sizeof(raw), 1, f) != 1)
                panic("RowOpTracePlayer: truncated trace at record %llu",
                      (unsigned long long)i);
            nr.start    = raw.start;
            nr.banks[0] = raw.banks;
            nr.banks[1] = 0;
            nr.lhs_bw = raw.lhs_bw;
            nr.rhs_bw = raw.rhs_bw;
            nr.kind   = raw.kind;
            nr.src    = raw.src;
            nr.dst    = raw.dst;
        } else {
            TraceRecord128 raw;
            if (fread(&raw, sizeof(raw), 1, f) != 1)
                panic("RowOpTracePlayer: truncated trace at record %llu",
                      (unsigned long long)i);
            nr.start    = raw.start;
            nr.banks[0] = raw.banks[0];
            nr.banks[1] = raw.banks[1];
            nr.lhs_bw = raw.lhs_bw;
            nr.rhs_bw = raw.rhs_bw;
            nr.kind   = raw.kind;
            nr.src    = raw.src;
            nr.dst    = raw.dst;
        }

        if (nr.kind == OP_MULI) {
            max_lhs     = std::max(max_lhs, nr.lhs_bw);
            max_rhs     = std::max(max_rhs, nr.rhs_bw);
            max_out     = std::max(max_out, nr.lhs_bw + nr.rhs_bw);
            max_partial = std::max(max_partial, nr.lhs_bw - 1);
        } else if (nr.kind == OP_ADDI) {
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_rhs = std::max(max_rhs, nr.rhs_bw);
            max_out = std::max(max_out, std::min(nr.lhs_bw, nr.rhs_bw));
        } else { // ROW_COPY: src=out[], dst=lhs[]
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_out = std::max(max_out, nr.lhs_bw);
        }
    }
    fclose(f);

    // --- Assign slot offsets ---
    //   18 control rows, then: lhs | rhs | out | partial | tmp(1) | carry(2)
    lhsBase     = SLOT_DATA_BASE;
    rhsBase     = lhsBase     + max_lhs;
    outBase     = rhsBase     + max_rhs;
    partialBase = outBase     + max_out;
    tmpBase     = partialBase + std::max(max_partial, 0);
    carryBase   = tmpBase     + 1;
    int totalSlots = carryBase + 2;

    inform("RowOpTracePlayer: %llu records, format=%d-bank, backend=%s, "
           "slots=%d (rows/bank)",
           (unsigned long long)hdr.num_records, total_banks,
           backend == BK_FCDRAM ? "fcdram" : "simdram", totalSlots);

    if (totalSlots >= ROWS_PER_SUBARRAY)
        panic("RowOpTracePlayer: %d slots exceed rows_per_subarray (%d)",
              totalSlots, ROWS_PER_SUBARRAY);

    // Process records in schedule start-time order so that same-start records
    // (which the scheduler runs concurrently) expand into a contiguous group.
    std::stable_sort(records.begin(), records.end(),
                     [](const NormRecord& a, const NormRecord& b) {
                         return a.start < b.start;
                     });

    // --- Expand records into PendingOps, tagging each with its start-time ---
    for (uint64_t i = 0; i < hdr.num_records; i++) {
        const NormRecord& r = records[i];

        // Build global bank list from 128-bit bitmask.
        std::vector<int> banks;
        for (int b = 0; b < total_banks; b++)
            if (r.banks[b >> 6] & (1ull << (b & 63)))
                banks.push_back(b);

        size_t before = pendingOps.size();
        switch (r.kind) {
          case OP_ADDI:
            expandAdd(r.lhs_bw, r.rhs_bw, banks);
            break;
          case OP_MULI:
            expandMul(r.lhs_bw, r.rhs_bw, banks);
            break;
          case OP_ROW_COPY:
            expandRowCopy(r.lhs_bw, r.src, r.dst);
            break;
          default:
            panic("RowOpTracePlayer: unknown op kind %u at record %llu",
                  r.kind, (unsigned long long)i);
        }
        for (size_t k = before; k < pendingOps.size(); k++)
            pendingOps[k].start = r.start;
    }

    // --- Build start-group boundaries (cumulative end index of each group) ---
    // Ops within a group share a start-time and are issued concurrently; a
    // dependency barrier separates consecutive groups.
    groupEnds.clear();
    for (size_t i = 0; i < pendingOps.size(); ) {
        int64_t s = pendingOps[i].start;
        size_t j = i + 1;
        while (j < pendingOps.size() && pendingOps[j].start == s) j++;
        groupEnds.push_back(j);
        i = j;
    }

    inform("RowOpTracePlayer: expanded to %llu row-op packets",
           (unsigned long long)pendingOps.size());
}

// ---------------------------------------------------------------------------
// startup: load trace then kick off the first send
// ---------------------------------------------------------------------------

void
RowOpTracePlayer::startup()
{
    loadTrace();

    if (!pendingOps.empty())
        schedule(&sendEvent, curTick());
}

// ---------------------------------------------------------------------------
// sendNextOp: issue (pump) all remaining row-ops of the current start-group,
// concurrently, until the memory back-pressures.  Ops of one start-group are
// independent (the scheduler placed them at the same time), so they are all in
// flight at once; the memory's per-bank/channel timing overlaps independent
// banks and serialises same-bank chains.  The barrier that gates the *next*
// group lives in recvTimingResp.
// ---------------------------------------------------------------------------

void
RowOpTracePlayer::sendNextOp()
{
    size_t groupEnd = groupEnds[curGroup];

    while (issueIdx < groupEnd) {
        const PendingOp& op = pendingOps[issueIdx];

        Addr dest = slotAddr(op.dest_slot, op.dest_bank);
        Addr src1 = slotAddr(op.src1_slot, op.src1_bank);
        Addr src2 = (op.op == Request::ROWCOPY) ? 0
                                                 : slotAddr(op.src2_slot, op.src2_bank);

        PacketPtr pkt = makeRowOpPacket(op.op, dest, src1, src2);

        if (!port.sendTimingReq(pkt)) {
            // Back-pressure: stash and resume from recvReqRetry.
            retryPkt = pkt;
            numRetries++;
            return;
        }

        issueIdx++;
        numPacketsSent++;
        if (!firstOpSeen) { firstOpSeen = true; firstOpTick = curTick(); }
    }
    // Whole group issued; wait for all its completions before the next group.
}

// ---------------------------------------------------------------------------
// Port callbacks
// ---------------------------------------------------------------------------

bool
RowOpTracePlayer::TraceMasterPort::recvTimingResp(PacketPtr pkt)
{
    return player.recvTimingResp(pkt);
}

void
RowOpTracePlayer::TraceMasterPort::recvReqRetry()
{
    player.recvReqRetry();
}

bool
RowOpTracePlayer::recvTimingResp(PacketPtr pkt)
{
    // Clean up the packet
    delete pkt->req;
    delete pkt;

    completedCount++;
    lastOpTick = curTick();   // completion tick of the row-op just finished

    // Advance only when the entire current start-group has retired: this is the
    // dependency barrier that keeps start-group G+1 strictly after group G.
    if (completedCount >= groupEnds[curGroup]) {
        if (completedCount >= pendingOps.size()) {
            rowOpMakespan = lastOpTick - firstOpTick;
            inform("RowOpTracePlayer: all %llu packets completed, exiting "
                   "(row-op makespan %llu ticks over %llu start-groups)",
                   (unsigned long long)pendingOps.size(),
                   (unsigned long long)(lastOpTick - firstOpTick),
                   (unsigned long long)groupEnds.size());
            exitSimLoop("RowOpTracePlayer: trace replay complete");
            return true;
        }
        curGroup++;                            // move to next start-group
        schedule(&sendEvent, curTick() + 1);   // issue it
    }
    return true;
}

void
RowOpTracePlayer::recvReqRetry()
{
    assert(retryPkt != nullptr);

    if (!port.sendTimingReq(retryPkt)) {
        // Still blocked; wait for the next retry callback.
        numRetries++;
        return;
    }

    // Stashed packet accepted: count it and resume issuing the current group.
    retryPkt = nullptr;
    issueIdx++;
    numPacketsSent++;
    if (!firstOpSeen) { firstOpSeen = true; firstOpTick = curTick(); }
    sendNextOp();
}

RowOpTracePlayer*
RowOpTracePlayerParams::create()
{
    return new RowOpTracePlayer(this);
}
