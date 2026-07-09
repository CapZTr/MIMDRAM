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
// The current CIMTRACE format has record_size == 48: a 128-bit bank bitmask
// (uint64_t banks[2], bits 0..127), so it addresses up to 128 banks.  It is the
// only format the Cinnamon compiler emits (94_simdram_schedule_runner.c /
// 96_fcdram_schedule_runner.c consume the same file).
//
// record_size == 32 is a legacy/obsolete format (uint32_t banks bitmask, bits
// 0..31) kept only so old traces still replay -- no current writer produces it.
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

// Legacy/obsolete 32-bank on-disk record (record_size == 32; no longer emitted)
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
      bankParallel(p->bank_parallel),
      slotOffset(parseBackend(p->backend) == BK_FCDRAM ? ROWS_PER_SUBARRAY : 0),
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
//   global_bank ∈ [0, total_banks)   (up to 128 for the current format;
//                                      total_banks = banksPerChannel * channels)
//   channel    = global_bank / banksPerChannel
//   local_bank = global_bank % banksPerChannel
//
//   slot_addr = base + channel * channelSize
//                    + (slot * banksPerChannel + local_bank) * rowStride
//
//   Examples for the 128-bank format (channels = 128 / banksPerChannel):
//   For DDR4 (banksPerChannel=32, 4 channels):
//     bank 0-31 → ch0 at base; bank 32-63 → ch1 at base+channelSize; …
//   For HBM2 (banksPerChannel=8, 16 channels):
//     bank 0-7 → ch0 at base; bank 8-15 → ch1 at base+channelSize; …
//   For HBM3 (banksPerChannel=16, 8 channels):
//     bank 0-15 → ch0 at base; bank 16-31 → ch1 at base+channelSize; …
// ---------------------------------------------------------------------------

Addr
RowOpTracePlayer::slotAddr(int slot, int global_bank) const
{
    int channel    = global_bank / banksPerChannel;
    int local_bank = global_bank % banksPerChannel;
    // slotOffset shifts all compute/data rows into the middle subarray for
    // the FCDRAM backend, so reference slots at slot ± ROWS_PER_SUBARRAY
    // land in the two neighbouring subarrays (raw slot may be negative for
    // the lower neighbour; the sum is always >= 0).
    int row = slot + slotOffset;
    assert(row >= 0);
    return baseAddr
         + (Addr)channel * channelSize
         + ((Addr)row * banksPerChannel + local_bank) * rowStride;
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
// Composite emitters (match C helpers in 94_simdram_schedule_runner.c)
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
// SIMDRAM execute_add  (mirrors C code in 94_simdram_schedule_runner.c)
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
// SIMDRAM execute_mul  (mirrors C code in 94_simdram_schedule_runner.c)
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
// FCDRAM backend (COTS DDR4 functionally-complete gates + FracDRAM MAJ)
//
// Mirrors 96_fcdram_schedule_runner.c.  ADDI/MULI keep the exact same
// bit-serial shift-add schedule as the SIMDRAM backend; only the bit-row AND
// and full-adder primitives change, now built from ROWCLONE (intra-subarray
// copy), AND_XSUB / OR_XSUB / NOT_XSUB (cross-subarray gates, FCDRAM
// HPCA'24), MAJ3 (intra-subarray simultaneous multi-row majority,
// FracDRAM/PULSAR) and FRAC (VDD/2 initialisation, FracDRAM).
//
// The cost model follows the mechanisms as characterised on real chips:
//
//  * Operand layout (paper §6.1): both data operands of an AND/OR live in
//    the COMPUTE subarray as a simultaneously-activated row pair; the
//    neighbouring REFERENCE subarray holds only constants (threshold row +
//    VDD/2 row).  Compute/data rows sit in the MIDDLE of 3 subarrays
//    (slotOffset), reference rows at slot ± ROWS_PER_SUBARRAY.
//  * Half-row coverage (paper §5 footnote 6): open-bitline neighbours share
//    only half of a row's sense amps, so covering a full bit-plane takes one
//    APA towards EACH neighbour — every cross-subarray gate emits 2 packets.
//  * Reference re-initialisation (paper §6.1.3): each AND/OR overwrites its
//    reference pair with the NAND/NOR byproduct, so before every gate the
//    threshold row is re-cloned from a per-subarray constant source
//    (C_1/C_0 mirror) and the VDD/2 row is re-FRAC'd, per side.
//  * Destructive MAJ (FracDRAM): restoration writes the majority value into
//    ALL simultaneously-activated rows and the activation group includes a
//    VDD/2 helper row.  Operands that must survive are staged into scratch
//    copies and the helper (SLOT_DCC1N) is re-FRAC'd before every MAJ3.
//
// Like the Ambit backend, this player models op *type/count/timing* only,
// not functional results.  Addressing remains a fiction in one respect: an
// XSUB packet names the com row and ONE mirror row, standing for the whole
// N:N activation group of that side (timing is one APA either way).  Not
// modelled (declared idealisations): per-chip N:N activation-pair coverage
// (paper Fig.5) and primitive success rates (paper Obs.3-19).
// ===========================================================================

// ROWCLONE: dst <- src, intra-subarray copy (same subarray after slotOffset;
// reference-side re-init clones pass matching ± RPS offsets on both slots).
void
RowOpTracePlayer::emitClone(int dst_slot, int src_slot,
                             const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::ROWCLONE,
                              dst_slot, b, src_slot, b, 0, b});
}

// FRAC: drive dst to VDD/2 via interrupted activations (FracDRAM).
void
RowOpTracePlayer::emitFrac(int dst_slot, const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::FRAC,
                              dst_slot, b, dst_slot, b, 0, b});
}

// AND_XSUB: {com_slot, SLOT_T1} hold the operand pair in the compute
// subarray; per neighbour side re-initialise the reference pair (VDD
// threshold row from the C_1 mirror + FRAC the VDD/2 row) then issue the
// APA.  Result lands in the compute pair (com_slot = AND, T1 clobbered).
void
RowOpTracePlayer::emitAndXsub(int com_slot, const std::vector<int>& banks)
{
    for (int side : {+ROWS_PER_SUBARRAY, -ROWS_PER_SUBARRAY}) {
        emitClone(com_slot + side, SLOT_C_1 + side, banks); // VDD threshold
        emitFrac (SLOT_T1  + side,                  banks); // VDD/2 row
        for (int b : banks)
            pendingOps.push_back({Request::AND_XSUB,
                                  com_slot, b,
                                  com_slot + side, b, 0, b});
    }
}

// OR_XSUB: same as AND but the threshold row is restored to GND (C_0 mirror).
void
RowOpTracePlayer::emitOrXsub(int com_slot, const std::vector<int>& banks)
{
    for (int side : {+ROWS_PER_SUBARRAY, -ROWS_PER_SUBARRAY}) {
        emitClone(com_slot + side, SLOT_C_0 + side, banks); // GND threshold
        emitFrac (SLOT_T1  + side,                  banks); // VDD/2 row
        for (int b : banks)
            pendingOps.push_back({Request::OR_XSUB,
                                  com_slot, b,
                                  com_slot + side, b, 0, b});
    }
}

// NOT_XSUB: dst = ~src through the shared sense-amp NOT gate; no reference
// constants needed, but half-row coverage still takes one APA per side.
void
RowOpTracePlayer::emitNotXsub(int dst_slot, const std::vector<int>& banks)
{
    for (int side : {+ROWS_PER_SUBARRAY, -ROWS_PER_SUBARRAY})
        for (int b : banks)
            pendingOps.push_back({Request::NOT_XSUB,
                                  dst_slot, b,
                                  dst_slot + side, b, 0, b});
}

// MAJ3: com = MAJ(com, s1, s2), intra-subarray simultaneous multi-row
// activation (FracDRAM/PULSAR); timing reuses the charge-sharing MAJ latency
// (maj3Bank).  Callers FRAC the VDD/2 helper row (SLOT_DCC1N) first and must
// treat s1/s2 as destroyed afterwards (see abstract_mem MAJ3).
void
RowOpTracePlayer::emitMaj3(int com_slot, int s1_slot, int s2_slot,
                            const std::vector<int>& banks)
{
    for (int b : banks)
        pendingOps.push_back({Request::MAJ3,
                              com_slot, b, s1_slot, b, s2_slot, b});
}

// execute_row_and:  out = lhs AND rhs
// Both operands are cloned into the compute-subarray activation pair
// {out, T1}; emitAndXsub then re-inits the reference pair and fires one APA
// per neighbour side.  8 packets per bit-row AND.
void
RowOpTracePlayer::emitRowAndFc(int lhs_slot, int rhs_slot, int out_slot,
                                const std::vector<int>& banks)
{
    emitClone  (out_slot, lhs_slot, banks);  // compute pair row 0 = lhs
    emitClone  (SLOT_T1,  rhs_slot, banks);  // compute pair row 1 = rhs
    emitAndXsub(out_slot,           banks);  // out = lhs AND rhs
}

// execute_row_add:  MAJ-based full adder using the COTS 3-input MAJ.
//   cout = MAJ(a, b, cin)
//   sum  = MAJ(~cout, cin, MAJ(a, b, ~cout))   ; a ^ b ^ cin
// MAJ is destructive (restoration writes the majority into all activated
// rows), so every MAJ3 operates on scratch copies of the operands that must
// survive (b, cin, ~cout), and the VDD/2 helper row (B_DCC1N) is re-FRAC'd
// before every MAJ3.  17 packets per full adder:
//   9 CLONE + 3 FRAC + 3 MAJ3 + 1 NOT (2 packets, one per neighbour side).
// Staging rows: B_T3=cout, B_T2=~cout, B_T0=inner, B_T1/B_DCC0=MAJ scratch.
// Aliasing cout==cin stays safe: cin is only read (via copies) before the
// final carry commit.
void
RowOpTracePlayer::emitRowAddFc(int lhs_slot, int rhs_slot, int out_slot,
                                int cin_slot, int cout_slot,
                                const std::vector<int>& banks)
{
    // cout = MAJ(a, b, cin) -> B_T3   (b, cin staged; copies die in the MAJ)
    emitClone(SLOT_T3,   lhs_slot, banks);               // B_T3   = a
    emitClone(SLOT_T1,   rhs_slot, banks);               // B_T1   = b
    emitClone(SLOT_DCC0, cin_slot, banks);               // B_DCC0 = cin
    emitFrac (SLOT_DCC1N,          banks);               // VDD/2 helper
    emitMaj3 (SLOT_T3, SLOT_T1, SLOT_DCC0, banks);       // B_T3 = cout

    // ~cout -> B_T2  (cross-subarray NOT, one APA per neighbour side)
    emitNotXsub(SLOT_T2, banks);                         // B_T2 = ~cout

    // inner = MAJ(a, b, ~cout) -> B_T0  (~cout staged so B_T2 survives)
    emitClone(SLOT_T0,   lhs_slot, banks);               // B_T0   = a
    emitClone(SLOT_T1,   rhs_slot, banks);               // B_T1   = b (fresh)
    emitClone(SLOT_DCC0, SLOT_T2,  banks);               // B_DCC0 = ~cout
    emitFrac (SLOT_DCC1N,          banks);
    emitMaj3 (SLOT_T0, SLOT_T1, SLOT_DCC0, banks);       // B_T0 = inner

    // sum = MAJ(~cout, cin, inner) -> out  (cin staged: may be the C_0
    // constant row or a live carry row, which the MAJ would destroy)
    emitClone(out_slot, SLOT_T2,  banks);                // out  = ~cout
    emitClone(SLOT_T1,  cin_slot, banks);                // B_T1 = cin
    emitFrac (SLOT_DCC1N,         banks);
    emitMaj3 (out_slot, SLOT_T1, SLOT_T0, banks);        // out = sum

    // commit carry last
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

    // Detect format from record_size; derive total bank count.  The 48-byte
    // record is the current format (up to 128 banks); the 32-byte record is a
    // legacy/obsolete format kept only so old traces still replay.
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

        // Build global bank list from 128-bit bitmask (ascending order).
        std::vector<int> banks;
        for (int b = 0; b < total_banks; b++)
            if (r.banks[b >> 6] & (1ull << (b & 63)))
                banks.push_back(b);

        // All-bank mode: an ADDI/MULI record runs the SAME row-op on the SAME
        // rows across every bank in its mask (SIMD), which real PuD hardware
        // issues as ONE all-bank broadcast command per channel -- one bank's
        // op-time, not one per bank.  Keep a single representative bank per
        // channel so bank count no longer inflates the makespan (channels are
        // independent DRAMCtrls and stay parallel).  Mirrors OptiPIM's
        // single_bank_opt.  `banks` is ascending, so banks of one channel are
        // contiguous and we can dedup channels with a running last-channel id.
        std::vector<int> collapsed;
        if (bankParallel) {
            int lastCh = -1;
            for (int b : banks) {
                int ch = b / banksPerChannel;
                if (ch != lastCh) { collapsed.push_back(b); lastCh = ch; }
            }
        }
        const std::vector<int>& compBanks = bankParallel ? collapsed : banks;

        size_t before = pendingOps.size();
        switch (r.kind) {
          case OP_ADDI:
            expandAdd(r.lhs_bw, r.rhs_bw, compBanks);
            break;
          case OP_MULI:
            expandMul(r.lhs_bw, r.rhs_bw, compBanks);
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
