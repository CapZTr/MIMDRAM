#include "mem/rowop_trace_player.hh"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "base/cprintf.hh"
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

enum OpKind : uint8_t { OP_MULI = 0, OP_ADDI = 1, OP_ROW_COPY = 2,
                        OP_RELU = 3, OP_ADDI_WIDE = 4, OP_XNOR = 5,
                        OP_RANGE_SCAN = 6, OP_MIN = 7, OP_MAX = 8,
                        OP_SUBI = 9 };

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
    if (name == "prada")   return BK_PRADA;
    fatal("RowOpTracePlayer: unknown backend '%s' (expected 'simdram', "
          "'fcdram' or 'prada')", name.c_str());
    return BK_SIMDRAM; // unreachable; silences -Wreturn-type
}

RowOpTracePlayer::RowOpTracePlayer(const RowOpTracePlayerParams* p)
    : MemObject(p),
      port("port", *this),
      perChannel(p->per_channel),
      traceFile(p->trace_file),
      baseAddr(p->base_addr),
      banksPerChannel(p->banks_per_channel),
      channelSize(p->channel_size),
      rowStride(p->row_stride),
      backend(parseBackend(p->backend)),
      singleBank(p->single_bank_opt),
      slotOffset(parseBackend(p->backend) == BK_FCDRAM ? ROWS_PER_SUBARRAY : 0),
      masterID(p->system->getMasterId(name())),
      curGroup(0), issueIdx(0), completedCount(0),
      retryPkt(nullptr),
      firstOpSeen(false), firstOpTick(0), lastOpTick(0),
      lhsBase(SLOT_DATA_BASE), rhsBase(0), outBase(0),
      partialBase(0), tmpBase(0), carryBase(0),
      sendEvent(this),
      chanSendEvent(this)
{
    // One master port per connected channel (VectorMasterPort "chan_port").
    for (int i = 0; i < p->port_chan_port_connection_count; ++i) {
        chanPorts.push_back(
            new ChanMasterPort(csprintf("%s.chan_port[%d]", name(), i),
                               *this, i));
    }
}

// ---------------------------------------------------------------------------
// Port
// ---------------------------------------------------------------------------

BaseMasterPort&
RowOpTracePlayer::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    if (if_name == "chan_port" && (size_t)idx < chanPorts.size())
        return *chanPorts[idx];
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

void RowOpTracePlayer::emitRdStream(int slot, int bank)
{
    pendingOps.push_back({Request::ROW_RD_STREAM,
                          slot, bank, slot, bank, 0, bank});
}

void RowOpTracePlayer::emitWrStream(int slot, int bank)
{
    pendingOps.push_back({Request::ROW_WR_STREAM,
                          slot, bank, slot, bank, 0, bank});
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
      case BK_PRADA:   expandAddPrada  (lhs_bw, rhs_bw, banks); break;
    }
}

void
RowOpTracePlayer::expandSub(int lhs_bw, int rhs_bw,
                             const std::vector<int>& banks)
{
    switch (backend) {
      case BK_SIMDRAM: expandSubSimdram(lhs_bw, rhs_bw, banks); break;
      case BK_PRADA:   expandSubPrada  (lhs_bw, rhs_bw, banks); break;
      case BK_FCDRAM:
        panic("RowOpTracePlayer: OP_SUBI is not implemented for the fcdram "
              "backend");
    }
}

void
RowOpTracePlayer::expandMul(int lhs_bw, int rhs_bw,
                             const std::vector<int>& banks)
{
    switch (backend) {
      case BK_SIMDRAM: expandMulSimdram(lhs_bw, rhs_bw, banks); break;
      case BK_FCDRAM:  expandMulFcdram (lhs_bw, rhs_bw, banks); break;
      case BK_PRADA:   expandMulPrada  (lhs_bw, rhs_bw, banks); break;
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

// ===========================================================================
// PRADA backend (PRADA, ICCAD'24: "A Processing-using-Memory Architecture for
// Commodity DRAM Devices with Enhanced Compatibility and Reliability").
//
// Mirrors 95_prada_schedule_runner.c.  ADDI/MULI keep the exact same bit-serial
// shift-add schedule as the SIMDRAM backend; only the bit-row AND and the
// full-adder change.  PRADA's distinctive mechanism (Sequential Row Activation)
// lets it build them from three row-ops instead of Ambit's AAP chains:
//
//   * TRA  (Triple Row Activation, A A As P): a 3-row charge-sharing majority
//     R = MAJ(a,b,c); with c preset to 0 this is a AND b (paper §2.2/§4.2).
//     Timing = ROWAAAP (aaapBank: tRAS + 2*tWLOV).  The same 3-activation
//     sequence also serves as a two-destination copy (As A A P), which the
//     paper uses to stage operands -- identical timing.
//   * N    (single-command NOT, As N A P): PRADA inverts on the sense amp with
//     no DCC and writes the result to a second row (paper §4.1/§5).  Timing =
//     ROWANAP (anapBank: tRCD + tNOT + tRAS).
//   * 5RA  (Five Row Activation, A A A A As P): a 5-row majority used for the
//     sum, S = MAJ(a,b,cin,~C,~C) (paper §5.2).  Timing = ROWAAAAAP
//     (aaaaapBank: tRAS + 4*tWLOV).
//
// No new gem5 primitive is required: ROWAAAP / ROWANAP / ROWAAAAAP already
// exist with these timings.  Like the other backends this models op
// type/count/timing only, not functional data.  A handful of control-row slots
// (SLOT_T0..SLOT_DCC1) are reused as generic per-bank scratch; PRADA does not
// use the Ambit magic-row (T0_T1_T2, ...) semantics.
// ===========================================================================

// ROWAAAP: 3-activation sequence (TRA majority or two-destination copy).
void RowOpTracePlayer::emitAAAP(int dst_slot, int src1_slot, int src2_slot,
                                 int bank)
{
    pendingOps.push_back({Request::ROWAAAP,
                          dst_slot, bank, src1_slot, bank, src2_slot, bank});
}

// ROWANAP: sense src, invert (single-command NOT), copy to dst.
void RowOpTracePlayer::emitANAP(int dst_slot, int src_slot, int bank)
{
    pendingOps.push_back({Request::ROWANAP,
                          dst_slot, bank, src_slot, bank, 0, bank});
}

// ROWAAAAAP: 5-activation 5-row majority (5RA).  Timing is fixed at 5
// activations regardless of how many rows are named, so packing dst + 2 src
// addresses is sufficient for the timing model.
void RowOpTracePlayer::emitAAAAAP(int dst_slot, int src1_slot, int src2_slot,
                                   int bank)
{
    pendingOps.push_back({Request::ROWAAAAAP,
                          dst_slot, bank, src1_slot, bank, src2_slot, bank});
}

// execute_row_and:  out = lhs AND rhs = MAJ(lhs, rhs, 0)  (TRA with c = 0).
// out is preset to 0 (from the C_0 constant row) so the TRA reads {0, lhs, rhs};
// lhs/rhs are staged into scratch so the operands survive (they are reused
// across the multiplier's partial products).  4 row-ops per bit-row AND.
void
RowOpTracePlayer::emitRowAndPrada(int lhs_slot, int rhs_slot, int out_slot,
                                   const std::vector<int>& banks)
{
    for (int b : banks) emitAAP (out_slot, SLOT_C_0,  b);  // out = 0
    for (int b : banks) emitAAP (SLOT_T0,  lhs_slot,  b);  // T0  = lhs
    for (int b : banks) emitAAP (SLOT_T1,  rhs_slot,  b);  // T1  = rhs
    for (int b : banks) emitAAAP(out_slot, SLOT_T0, SLOT_T1, b); // out = MAJ(0,lhs,rhs)
}

// execute_row_add:  bit-serial full adder, PRADA style (paper §5.2).
//   carry-out C = MAJ(a, b, cin)                 -> 1 TRA  (ROWAAAP)
//   ~C                                            -> 1 NOT  (ROWANAP)
//   sum       S = MAJ(a, b, cin, ~C, ~C)          -> 1 5RA  (ROWAAAAAP)
// Staging (a, b consumed by the TRA are single-use here, but the sum needs
// a, b, cin again, so fresh copies are preserved first):
//   out <- a ; T2 <- b ; {cout, T3} <- cin  (cin duplicated for TRA and 5RA).
// The TRA runs in place on cout (= cin) so cout holds the carry-out; the NOT
// inverts the carry copy left in the lhs row, leaving cout = C intact.  Total
// 6 row-ops per bit vs SIMDRAM's 9 -- 2 AAP + 2 AAAP + 1 ANAP + 1 AAAAAP.
void
RowOpTracePlayer::emitRowAddPrada(int lhs_slot, int rhs_slot, int out_slot,
                                   int cin_slot, int cout_slot,
                                   const std::vector<int>& banks)
{
    for (int b : banks) emitAAP  (out_slot,  lhs_slot, b);            // out = a
    for (int b : banks) emitAAP  (SLOT_T2,   rhs_slot, b);            // T2  = b
    for (int b : banks) emitAAAP (cout_slot, cin_slot, SLOT_T3, b);   // cout=cin, T3=cin
    for (int b : banks) emitAAAP (cout_slot, lhs_slot, rhs_slot, b);  // cout = MAJ(cin,a,b) = C
    for (int b : banks) emitANAP (SLOT_DCC0, lhs_slot, b);            // DCC0 = ~C (lhs holds C)
    for (int b : banks) emitAAAAAP(out_slot, SLOT_T2, SLOT_T3, b);    // out = MAJ(a,b,cin,~C,~C)
}

// PRADA execute_add — ripple-carry adder (one full adder per bit); the carry
// ripples in place through SLOT_DCC1 (cin == cout each bit).
void
RowOpTracePlayer::expandAddPrada(int lhs_bw, int rhs_bw,
                                  const std::vector<int>& banks)
{
    int bw = std::min(lhs_bw, rhs_bw);

    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_0, b);   // carry := 0

    for (int j = 0; j < bw; j++)
        emitRowAddPrada(lhsBase + j, rhsBase + j, outBase + j,
                        SLOT_DCC1, SLOT_DCC1, banks);
}

// PRADA execute_mul — shift-add multiplier (schedule identical to SIMDRAM;
// only emitRowAnd/emitRowAdd are the PRADA variants).
void
RowOpTracePlayer::expandMulPrada(int lhs_bw, int rhs_bw,
                                  const std::vector<int>& banks)
{
    // Phase 1: init out[0] and partial[] from rhs[0]
    emitRowAndPrada(lhsBase + 0, rhsBase + 0, outBase + 0, banks);

    for (int i = 0; i < lhs_bw - 1; i++)
        emitRowAndPrada(lhsBase + i + 1, rhsBase + 0, partialBase + i, banks);

    for (int b : banks) emitAAP(carryBase + 1, SLOT_C_0, b);   // carry[1] := 0

    // Phase 2: accumulate rhs[1..rhs_bw-2]
    for (int i = 0; i < rhs_bw - 1; i++) {
        emitRowAndPrada(lhsBase + 0, rhsBase + i, tmpBase + 0, banks);
        emitRowAddPrada(tmpBase + 0, partialBase + 0, outBase + i,
                        SLOT_C_0,    carryBase + 0,   banks);

        for (int j = 1; j < lhs_bw - 1; j++) {
            emitRowAndPrada(lhsBase + j, rhsBase + i, tmpBase + 0, banks);
            emitRowAddPrada(tmpBase + 0, partialBase + j, partialBase + j - 1,
                            carryBase + 0, carryBase + 0, banks);
        }

        emitRowAndPrada(lhsBase + lhs_bw - 1, rhsBase + i, tmpBase + 0, banks);
        emitRowAddPrada(tmpBase + 0, carryBase + 0, partialBase + lhs_bw - 2,
                        carryBase + 1, carryBase + 1, banks);
    }

    // Phase 3: final row rhs[rhs_bw-1]
    emitRowAndPrada(lhsBase + 0, rhsBase + rhs_bw - 1, tmpBase + 0, banks);

    for (int i = 1; i < lhs_bw - 1; i++) {
        emitRowAndPrada(lhsBase + i, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
        emitRowAddPrada(tmpBase + 0, partialBase + i - 1, outBase + rhs_bw - 1 + i,
                        carryBase + 0, carryBase + 0, banks);
    }

    emitRowAndPrada(lhsBase + lhs_bw - 1, rhsBase + rhs_bw - 1, tmpBase + 0, banks);
    emitRowAddPrada(tmpBase + 0, carryBase + 1, outBase + lhs_bw + rhs_bw - 2,
                    carryBase + 0, outBase + lhs_bw + rhs_bw - 1, banks);
}

// ===========================================================================
// PRADA versions of the five non-arithmetic ops.
//
// Two properties of PRADA's substrate shape every sequence below, and they cut
// in opposite directions against Ambit:
//
//   + A TRA is ONE command for a 3-row majority, where Ambit needs an AAP to
//     stage each operand into a magic row plus an AP.  So a majority whose
//     operands are already in place is far cheaper here.
//   - There is NO dual-contact cell.  Ambit gets ~a for free by reading DCC0N
//     after writing DCC0; PRADA must spend a NOT (ROWANAP) per complement.
//
// The operand-staging convention is inherited from the existing PRADA and:
// a TRA is destructive in ALL THREE named rows (charge sharing drives the
// majority back into each), so anything that must survive is copied into
// scratch first.  The five sequences below follow emitRowAndPrada in staging
// the caller's input slices rather than naming them in a TRA, so an input row
// is only ever read.  (emitRowAddPrada, which predates them, does name its
// operand rows and so consumes them; that is harmless there because
// expandMulPrada only ever feeds it scratch, and it is a timing model either
// way, but do not copy the pattern.)
//
// Slots SLOT_T0..SLOT_DCC1N are plain scratch rows here (PRADA has no magic
// multi-row semantics), which gives eight of them; SLOT_C_0/SLOT_C_1 hold the
// constant 0/1 rows a majority needs to degenerate into AND (third row 0) or
// OR (third row 1).
// ===========================================================================

// out = lhs OR rhs = MAJ(lhs, rhs, 1) -- the dual of emitRowAndPrada, with the
// third row preset to 1 instead of 0.  4 row-ops.
void
RowOpTracePlayer::emitRowOrPrada(int lhs_slot, int rhs_slot, int out_slot,
                                 const std::vector<int>& banks)
{
    for (int b : banks) emitAAP (out_slot, SLOT_C_1,  b);  // out = 1
    for (int b : banks) emitAAP (SLOT_T0,  lhs_slot,  b);  // T0  = lhs
    for (int b : banks) emitAAP (SLOT_T1,  rhs_slot,  b);  // T1  = rhs
    for (int b : banks) emitAAAP(out_slot, SLOT_T0, SLOT_T1, b);
}

// Widening add, PRADA: expandAddPrada plus the surviving carry-out row.
// Cost 6*bw + 2 (against SIMDRAM's 8*bw + 2).  Same reason the store cannot be
// skipped: the next add's carry initialisation overwrites SLOT_DCC1.
void
RowOpTracePlayer::expandAddWidePrada(int bw, const std::vector<int>& banks)
{
    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_0, b);   // carry := 0

    for (int j = 0; j < bw; j++)
        emitRowAddPrada(lhsBase + j, rhsBase + j, outBase + j,
                        SLOT_DCC1, SLOT_DCC1, banks);

    for (int b : banks) emitAAP(outBase + bw, SLOT_DCC1, b);
}

// ReLU, PRADA: out[i] = lhs[i] AND ~sign.  The mask costs a single command
// here -- PRADA's NOT writes the inverted sense-amp value straight into a
// second row, with no dual-contact cell involved -- and then it is ANDed into
// every bit row with the standard PRADA AND (4 ops).  emitRowAndPrada touches
// only out/T0/T1, so the mask parked in T3 survives the whole loop.
//
// Cost 4*bw + 1, against SIMDRAM's 4*bw + 2.
void
RowOpTracePlayer::expandReluPrada(int bw, const std::vector<int>& banks)
{
    for (int b : banks) emitANAP(SLOT_T3, lhsBase + bw - 1, b);  // T3 = ~sign

    for (int i = 0; i < bw; i++)
        emitRowAndPrada(lhsBase + i, SLOT_T3, outBase + i, banks);
}

// XNOR, PRADA: (a OR ~b) AND (~a OR b), three majorities like the Ambit
// version -- but each complement now costs a NOT, and each majority's operands
// must be staged because the TRA consumes all three of its rows.  Per bit:
//
//   1. ANAP(T0,  a[i])          T0   = ~a
//   2. AAP (T1,  b[i])          T1   =  b      (input rows stay intact)
//   3. AAP (T2,  C_1)           T2   =  1
//   4. AAAP(T2,  T0, T1)        T2   = MAJ(1,~a,b) = ~a|b
//   5. ANAP(T3,  b[i])          T3   = ~b
//   6. AAP (DCC0, a[i])         DCC0 =  a
//   7. AAP (DCC1, C_1)          DCC1 =  1
//   8. AAAP(DCC1, T3, DCC0)     DCC1 = MAJ(1,~b,a) = a|~b
//   9. AAP (out[i], C_0)        out  =  0
//  10. AAAP(out[i], T2, DCC1)   out  = MAJ(0, ~a|b, a|~b) = XNOR
//
// Step 4 clobbers T0/T1/T2 (all three then hold ~a|b), which is why steps 5-8
// rebuild the second OR's operands in different rows; step 10 reads the two
// OR results out of T2 and DCC1, the rows each majority was accumulated in.
// Nothing is carried between bit positions, so this loops safely at any width.
//
// Cost 10 per bit, against SIMDRAM's 7: PRADA pays 2 NOTs per bit where the
// dual-contact cells hand Ambit both complements for free, and it cannot use
// Ambit's trick of computing both ORs with two bare APs over pre-loaded magic
// rows.  XNOR is the one op in this set where PRADA is clearly worse -- worth
// stating plainly, since it is the BNN multiply.
void
RowOpTracePlayer::expandXnorPrada(int bw, const std::vector<int>& banks)
{
    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitANAP(SLOT_T0,   lhsBase + i, b);
        for (int b : banks) emitAAP (SLOT_T1,   rhsBase + i, b);
        for (int b : banks) emitAAP (SLOT_T2,   SLOT_C_1,    b);
        for (int b : banks) emitAAAP(SLOT_T2,   SLOT_T0, SLOT_T1, b);
        for (int b : banks) emitANAP(SLOT_T3,   rhsBase + i, b);
        for (int b : banks) emitAAP (SLOT_DCC0, lhsBase + i, b);
        for (int b : banks) emitAAP (SLOT_DCC1, SLOT_C_1,    b);
        for (int b : banks) emitAAAP(SLOT_DCC1, SLOT_T3, SLOT_DCC0, b);
        for (int b : banks) emitAAP (outBase + i, SLOT_C_0,  b);
        for (int b : banks) emitAAAP(outBase + i, SLOT_T2, SLOT_DCC1, b);
    }
}

// BitWeaving/V BETWEEN range scan, PRADA.  Same single-bit recurrence as the
// Ambit version (R = ">= lower", S = "<= upper", advanced by one majority
// each), and PRADA's TRA is a particularly good fit: naming the accumulator
// row itself as one of the three rows updates it IN PLACE, since the majority
// is driven back into every activated row.  Per bit:
//
//   1. AAP (T0, col[j])       stage v            (column rows stay intact)
//   2. ANAP(T1, col[j])       ~v
//   3. AAP (T2, C_lower[j])   selector for R     -- branch-dependent
//   4. AAAP(DCC0, T0, T2)     R = MAJ(R, v, sel_lo)
//   5. AAP (T3, C_upper[j])   selector for S     -- branch-dependent
//   6. AAAP(DCC1, T1, T3)     S = MAJ(S, ~v, sel_hi)
//
// As in the Ambit version the cost is branch-independent: every one of the
// four (lower[j], upper[j]) cases emits these same six commands and differs
// only in which constant row steps 3 and 5 read, so emitting one
// representative branch is exact rather than approximate, and the compiler can
// price the op from the column width alone.
//
// Cost 6*bw + 6: 2 to seed both chains, 6 per bit, and 4 for the final
// mask = R AND S.  Against SIMDRAM's 4*bw + 5 -- Ambit wins here because one
// AP advances a chain whose operands are already in its magic rows, while
// PRADA must re-stage v and rebuild ~v every bit.
//
// No early termination, matching the reference implementations and the Ambit
// path; this is an upper bound on the scan cost.
void
RowOpTracePlayer::expandRangeScanPrada(int bw, const std::vector<int>& banks)
{
    for (int b : banks) emitAAP(SLOT_DCC0, SLOT_C_1, b);   // R := 1
    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_1, b);   // S := 1

    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP (SLOT_T0,   lhsBase + i, b);
        for (int b : banks) emitANAP(SLOT_T1,   lhsBase + i, b);
        for (int b : banks) emitAAP (SLOT_T2,   SLOT_C_1,    b);
        for (int b : banks) emitAAAP(SLOT_DCC0, SLOT_T0, SLOT_T2, b);
        for (int b : banks) emitAAP (SLOT_T3,   SLOT_C_0,    b);
        for (int b : banks) emitAAAP(SLOT_DCC1, SLOT_T1, SLOT_T3, b);
    }

    // mask = R AND S.  R and S are in scratch, so the standard PRADA AND
    // applies unchanged (it stages both operands and preserves them).
    emitRowAndPrada(SLOT_DCC0, SLOT_DCC1, outBase + 0, banks);
}

// Bit-serial min/max, PRADA: compare, then select.
//
// Phase 1 -- G = (a > b) = MAJ3(a[j], ~b[j], G), LSB to MSB.  Three commands
// per bit, exactly as on Ambit, but for a different reason: PRADA spends one
// on the complement of b and gets the majority itself for free in the TRA,
// where Ambit spends two AAPs staging operands and one AP.  G accumulates in
// SLOT_DCC0N -- in PRADA that is simply another scratch row, no dual-contact
// semantics -- and the TRA updates it in place.
//
// Phase 2 -- out[i] = mux(G, x[i], y[i]) = (G AND x) OR (~G AND y), three
// majorities, 10 commands per bit:
//
//   1. AAP (DCC0, G)          a copy of G to feed the TRA, which consumes it
//   2. ANAP(DCC1, G)          ~G
//   3. AAP (T0, C_0)          0
//   4. AAP (T1, x[i])         stage x
//   5. AAAP(T0, T1, DCC0)     T0 = MAJ(0, x, G)  = G AND x
//   6. AAP (T2, C_0)          0
//   7. AAP (T3, y[i])         stage y
//   8. AAAP(T2, T3, DCC1)     T2 = MAJ(0, y, ~G) = ~G AND y
//   9. AAP (out[i], C_1)      1
//  10. AAAP(out[i], T0, T2)   out = MAJ(1, G&x, ~G&y) = the mux
//
// Steps 1-2 exist because a TRA destroys all three of its rows: G has to
// survive every remaining bit, so it is never named directly.
//
// Total 13*bw + 1, one command under SIMDRAM's 13*bw + 2 -- the two substrates
// happen to land on the same per-bit cost for this op.  max and min differ
// only in which operand each mux branch takes.
void
RowOpTracePlayer::expandMinMaxPrada(int bw, const std::vector<int>& banks,
                                   bool isMax)
{
    const int gSlot = SLOT_DCC0N;

    // --- Phase 1: G = (a > b), seeded false so equal operands select b.
    for (int b : banks) emitAAP(gSlot, SLOT_C_0, b);
    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP (SLOT_T0, lhsBase + i, b);
        for (int b : banks) emitANAP(SLOT_T1, rhsBase + i, b);
        for (int b : banks) emitAAAP(gSlot,   SLOT_T0, SLOT_T1, b);
    }

    // --- Phase 2: out[i] = mux(G, x[i], y[i]).
    const int xBase = isMax ? lhsBase : rhsBase;
    const int yBase = isMax ? rhsBase : lhsBase;
    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP (SLOT_DCC0,   gSlot,      b);
        for (int b : banks) emitANAP(SLOT_DCC1,   gSlot,      b);
        for (int b : banks) emitAAP (SLOT_T0,     SLOT_C_0,   b);
        for (int b : banks) emitAAP (SLOT_T1,     xBase + i,  b);
        for (int b : banks) emitAAAP(SLOT_T0,     SLOT_T1, SLOT_DCC0, b);
        for (int b : banks) emitAAP (SLOT_T2,     SLOT_C_0,   b);
        for (int b : banks) emitAAP (SLOT_T3,     yBase + i,  b);
        for (int b : banks) emitAAAP(SLOT_T2,     SLOT_T3, SLOT_DCC1, b);
        for (int b : banks) emitAAP (outBase + i, SLOT_C_1,   b);
        for (int b : banks) emitAAAP(outBase + i, SLOT_T0, SLOT_T2, b);
    }
}

// ---------------------------------------------------------------------------
// Bit-serial subtract: out = lhs - rhs = lhs + ~rhs + 1.
//
// The same full-adder chain as expandAddSimdram, with two changes:
//   * the carry is initialised to ONE (read C_1 instead of C_0), which is the
//     +1 of the two's complement, and costs the same one AAP;
//   * every read of the subtrahend's row becomes a read of its complement.
//
// The complement has to be materialised rather than read on the fly, because
// the adder reads rhs TWICE per bit -- once into the majority that forms the
// carry-out and once into the sum -- and a dual-contact cell only hands back
// the complement of whatever was last written to it, which for the carry path
// is not rhs.  So each bit stages rhs into DCC0 once (1 AAP) and then reads
// DCC0N in both places, for 9 commands per bit against the adder's 8.
//
// Cost 9*bw + 1, matching getComputeLatency(SubIOp) on this backend.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandSubSimdram(int lhs_bw, int rhs_bw,
                                   const std::vector<int>& banks)
{
    int bw = std::min(lhs_bw, rhs_bw);

    // init carry = 1 (the +1 of ~rhs + 1)
    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_1, b);

    for (int j = 0; j < bw; j++) {
        int lhs_slot = lhsBase + j;
        int rhs_slot = rhsBase + j;
        int out_slot = outBase + j;

        // DCC0 = rhs, so DCC0N reads ~rhs for the rest of this bit step.
        for (int b : banks) emitAAP(SLOT_DCC0,       rhs_slot,        b);

        for (int b : banks) emitAAP(SLOT_T0_T1_T2,  SLOT_DCC1,      b);
        for (int b : banks) emitAAP(SLOT_T2_T3,     lhs_slot,        b);
        for (int b : banks) emitAAP(SLOT_DCC1,      SLOT_DCC0N,      b);
        for (int b : banks) emitAP (SLOT_DCC1_T0_T3,                  b);
        for (int b : banks) emitAAP(SLOT_T0_T3,     SLOT_DCC1N,      b);
        for (int b : banks) emitAP (SLOT_T0_T1_T2,                    b);
        for (int b : banks) emitAAP(SLOT_T1,        SLOT_DCC0N,      b);
        for (int b : banks) emitAAP(out_slot,       SLOT_T1_T2_T3,   b);
    }
}

// PRADA subtract: the PRADA full adder with the carry seeded to one and the
// subtrahend inverted by PRADA's single-command NOT into T1 -- which
// emitRowAddPrada does not write (it uses out/T2/T3/DCC0 and the carry slot),
// so the inverted row survives the adder step.  7 commands per bit against the
// adder's 6, i.e. 7*bw + 1.  Note this is a smaller penalty than SIMDRAM's,
// because a NOT here is one command while Ambit needs a full AAP through a
// dual-contact cell.
void
RowOpTracePlayer::expandSubPrada(int lhs_bw, int rhs_bw,
                                 const std::vector<int>& banks)
{
    int bw = std::min(lhs_bw, rhs_bw);

    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_1, b);   // carry := 1

    for (int j = 0; j < bw; j++) {
        for (int b : banks) emitANAP(SLOT_T1, rhsBase + j, b);   // T1 = ~rhs
        emitRowAddPrada(lhsBase + j, SLOT_T1, outBase + j,
                        SLOT_DCC1, SLOT_DCC1, banks);
    }
}

// ---------------------------------------------------------------------------
// Widening bit-serial add: out[0..bw] = lhs[0..bw-1] + rhs[0..bw-1].
//
// Identical to expandAddSimdram except that the final carry -- which lives in
// SLOT_DCC1 and survives across bit steps -- is stored into out[bw] instead of
// being discarded.  That one extra AAP is what buys the extra result bit, and
// it cannot be skipped: the next addition's carry initialisation overwrites
// DCC1, so a consumer could not read it from there.
//
// Cost is 8*bw + 2, matching getComputeLatency(AddIFullOp) exactly.  Compare
// with pre-extending both operands and using a truncating add, which costs
// 8*(bw+1) + 1 -- for a 64-input popcount tree that is 1527 AAP against 1086.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandAddWide(int bw, const std::vector<int>& banks)
{
    if (backend == BK_PRADA) { expandAddWidePrada(bw, banks); return; }
    if (backend != BK_SIMDRAM)
        panic("RowOpTracePlayer: OP_ADDI_WIDE is not implemented for the "
              "fcdram backend");

    for (int b : banks) emitAAP(SLOT_DCC1, SLOT_C_0, b);   // carry = 0

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

    // The extra result row: store the surviving carry-out.
    for (int b : banks) emitAAP(outBase + bw, SLOT_DCC1, b);
}

// ---------------------------------------------------------------------------
// Bit-serial ReLU (predicated zero): out[i] = lhs[i] AND ~sign(lhs).
//
// For a two's-complement value the sign bit is the top data row, so ReLU needs
// no comparator and no multiplexer: invert that one row once, then AND it into
// every bit row.  Cost is exactly 2 + 4*bw AAP -- 2 for the inversion through
// the dual-contact cell (write DCC1, read its negated output DCC1N) and 4 per
// bit for execute_row_and.  This matches getComputeLatency()'s 4*bw + 2 in the
// Cinnamon scheduler exactly, which is the invariant the whole comparison
// rests on.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandRelu(int bw, const std::vector<int>& banks)
{
    if (backend == BK_PRADA) { expandReluPrada(bw, banks); return; }
    if (backend != BK_SIMDRAM)
        panic("RowOpTracePlayer: OP_RELU is not implemented for the fcdram "
              "backend; its cross-subarray AND/NOT operand staging differs "
              "and has not been validated for this op");

    // mask = ~sign, via the dual-contact cell.
    for (int b : banks) emitAAP(SLOT_DCC1, lhsBase + bw - 1, b); // DCC1 = sign
    for (int b : banks) emitAAP(SLOT_T3,   SLOT_DCC1N,       b); // T3 = ~sign

    // out[i] = lhs[i] AND mask.  emitRowAnd uses T0/T1/T2 only, so the mask
    // parked in T3 survives every iteration.
    for (int i = 0; i < bw; i++)
        emitRowAnd(lhsBase + i, SLOT_T3, outBase + i, banks);
}

// ---------------------------------------------------------------------------
// XNOR: the binary-neural-network multiply.  With +/-1 encoded in one bit
// (0 -> -1, 1 -> +1), multiplying two binary values IS the equality predicate,
// so a whole K-element binary dot product becomes one XNOR plus a popcount.
//
// XNOR(a,b) = (a OR ~b) AND (~a OR b) -- three majority gates.  Each dual
// contact cell supplies an operand and its complement simultaneously, so both
// OR terms are computed in place with a bare AP and no extra copies:
//
//   1. AAP(DCC0N_T0, lhs[i])  DCC0N = a, T0 = a   =>  DCC0 reads ~a
//   2. AAP(DCC1N_T1, rhs[i])  DCC1N = b, T1 = b   =>  DCC1 reads ~b
//   3. AAP(T2_T3,    C_1)     T2 = T3 = 1
//   4. AP (DCC0_T1_T2)        maj(~a, b, 1) = ~a|b  -> DCC0, T1, T2
//   5. AP (DCC1_T0_T3)        maj(~b, a, 1) =  a|~b -> DCC1, T0, T3
//   6. AAP(T2,       C_0)     T2 = 0   (~a|b survives in T1)
//   7. AAP(out[i],   T0_T1_T2) maj(a|~b, ~a|b, 0) = XNOR
//
// Step 5 reads only rows step 4 did not touch (it writes DCC0/T1/T2, step 5
// reads DCC1/T0/T3), and step 7 reads T0 from step 5, T1 from step 4 and T2
// from step 6 -- no step clobbers a row a later step still needs, and nothing
// is carried between bit positions, so the loop is safe at any width.
//
// Cost is exactly 7 commands per bit, counting AAP and AP alike -- the same
// convention expandAddSimdram is priced under (6 AAP + 2 AP per bit = 8n+1).
// This matches getComputeLatency()'s 7*n in the Cinnamon scheduler exactly,
// which is the invariant the whole comparison rests on.
//
// XOR would be this identical sequence with C_0 and C_1 swapped in steps 3
// and 6 (two ANDs combined by an OR instead of the reverse); it is not
// emitted today, so it has no record kind.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandXnor(int bw, const std::vector<int>& banks)
{
    if (backend == BK_PRADA) { expandXnorPrada(bw, banks); return; }
    if (backend != BK_SIMDRAM)
        panic("RowOpTracePlayer: OP_XNOR is not implemented for the fcdram "
              "backend; its cross-subarray majority-gate operand staging "
              "differs and has not been validated for this op");

    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP(SLOT_DCC0N_T0, lhsBase + i,   b);
        for (int b : banks) emitAAP(SLOT_DCC1N_T1, rhsBase + i,   b);
        for (int b : banks) emitAAP(SLOT_T2_T3,    SLOT_C_1,      b);
        for (int b : banks) emitAP (SLOT_DCC0_T1_T2,              b);
        for (int b : banks) emitAP (SLOT_DCC1_T0_T3,              b);
        for (int b : banks) emitAAP(SLOT_T2,       SLOT_C_0,      b);
        for (int b : banks) emitAAP(outBase + i,   SLOT_T0_T1_T2, b);
    }
}

// ---------------------------------------------------------------------------
// BitWeaving/V BETWEEN range scan: lower < value < upper over a bw-bit
// bit-sliced column, producing one mask row.  Models 90_bitweave-plus.c.
//
// Two comparison chains advance together, one majority gate each, via the
// single-bit recurrence (LSB to MSB, R = ">= lower", S = "<= upper"):
//
//   lower[j]==0 -> R = R OR  v[j]     upper[j]==1 -> S = S OR  ~v[j]
//   lower[j]==1 -> R = R AND v[j]     upper[j]==0 -> S = S AND ~v[j]
//
// MAJ3(sel, v, acc) is OR when sel==1 and AND when sel==0, so one AP per
// chain advances it; the dual-contact cell hands back ~v for free.  Per bit:
//
//   1. AAP(DCC0N_T0, col[j])   load v      => DCC0 reads ~v
//   2. AAP(<sel rows>, C_0|C_1) the ONLY branch-dependent command
//   3. AP (DCC1_T0_T3)         advance R
//   4. AP (DCC0_T1_T2)         advance S
//
// Each accumulator is held redundantly in three rows; step 2 clobbers a
// different pair depending on the branch, always leaving one live copy, and
// the APs broadcast the majority back into all three.  Which rows hold the
// selector and which hold the accumulator therefore SWAP between branches --
// that is what lets one AP serve both OR and AND.
//
// Cost is 4 commands per bit regardless of branch: all four cases emit one
// AAP for the selector, differing only in which constant row is read and
// which group is written.  The scanned constants therefore do not affect
// timing, which is why the compiler prices this from the bit width alone and
// why emitting one representative branch here is exact.  Total 4*bw + 5,
// matching getComputeLatency()'s 4n + 5.
//
// NOTE vs the reference: 90_bitweave-plus.c seeds only DCC1 and T0/T1/T2, and
// reads T3 as an accumulator on its first iteration when lower[0]==upper[0] --
// T3 is never initialised there.  It is a pure timing microbenchmark whose
// result is never checked, so that gap does not affect its numbers, but the
// sequence as written is not correct.  The extra AAP(T3, C_1) below closes it
// for one command, which is why this is 4n+5 and not the reference's 4n+4.
//
// No early termination, matching all three reference implementations (they
// scan every bit plane unconditionally).  Real BitWeaving/V can stop once
// every value is resolved, which is data dependent and cannot be expressed in
// a static trace -- so this is an upper bound on the scan cost.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandRangeScan(int bw, const std::vector<int>& banks)
{
    if (backend == BK_PRADA) { expandRangeScanPrada(bw, banks); return; }
    if (backend != BK_SIMDRAM)
        panic("RowOpTracePlayer: OP_RANGE_SCAN is not implemented for the "
              "fcdram backend; its cross-subarray majority-gate operand "
              "staging differs and has not been validated for this op");

    // Seed both chains to 1 (inclusive BETWEEN): R in DCC1, S in T2.
    for (int b : banks) emitAAP(SLOT_DCC1,     SLOT_C_1, b);
    for (int b : banks) emitAAP(SLOT_T0_T1_T2, SLOT_C_1, b);
    for (int b : banks) emitAAP(SLOT_T3,       SLOT_C_1, b);

    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP(SLOT_DCC0N_T0, lhsBase + i, b);
        for (int b : banks) emitAAP(SLOT_DCC1N_T1, SLOT_C_0,    b);
        for (int b : banks) emitAP (SLOT_DCC1_T0_T3,            b);
        for (int b : banks) emitAP (SLOT_DCC0_T1_T2,            b);
    }

    // mask = R AND S = maj(0, T2, T3).
    for (int b : banks) emitAAP(SLOT_T1,     SLOT_C_0,      b);
    for (int b : banks) emitAAP(outBase + 0, SLOT_T1_T2_T3, b);
}

// ---------------------------------------------------------------------------
// Bit-serial min/max: compare, then select.  The two differ only in which
// operand each mux branch takes, so one implementation serves both.
//
// Phase 1 -- compare, G = (a > b), LSB to MSB.  The whole comparator is ONE
// majority gate per bit:
//
//     G_new = MAJ3(a[j], ~b[j], G_prev)
//
//   a=0,b=0 -> maj(0,1,G) = G   (equal, carry the lower-bit verdict)
//   a=1,b=1 -> maj(1,0,G) = G   (equal, carry)
//   a=1,b=0 -> maj(1,1,G) = 1   (a > b)
//   a=0,b=1 -> maj(0,0,G) = 0   (a < b)
//
// G is parked in T3 and survives each round: the next iteration's AAPs
// clobber T0 (G still in DCC1 and T3) and DCC1 (G still in T3).  3 per bit.
//
// Phase 2 -- select.  mux(G,x,y) = (G AND x) OR (~G AND y)
//                               = MAJ(MAJ(G,x,0), MAJ(~G,y,0), 1),
// three majority gates, 10 commands per bit.  G is first parked in DCC0 so
// that both G and ~G are readable for free while T0..T3 are reused as scratch.
//
//   max = mux(G, a, b)   min = mux(G, b, a)
//
// Total 13*bw + 2, matching getComputeLatency()'s 13n + 2.
//
// Not a fused compare-exchange: sharing one comparison between a min and a
// max would save 3*bw of 26*bw (12%) at the cost of a two-result op, which is
// not worth it for a component that is a third of the KNN kernel.
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandMinMax(int bw, const std::vector<int>& banks,
                               bool isMax)
{
    if (backend == BK_PRADA) { expandMinMaxPrada(bw, banks, isMax); return; }
    if (backend != BK_SIMDRAM)
        panic("RowOpTracePlayer: OP_MIN/OP_MAX are not implemented for the "
              "fcdram backend; its cross-subarray majority-gate operand "
              "staging differs and has not been validated for these ops");

    // --- Phase 1: G = (a > b), seeded false so that equal operands select b.
    for (int b : banks) emitAAP(SLOT_T3, SLOT_C_0, b);
    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP(SLOT_T0,       lhsBase + i, b);
        for (int b : banks) emitAAP(SLOT_DCC1N_T1, rhsBase + i, b);
        for (int b : banks) emitAP (SLOT_DCC1_T0_T3,            b);
    }

    // Park the verdict where its complement is free.
    for (int b : banks) emitAAP(SLOT_DCC0, SLOT_T3, b);

    // --- Phase 2: out[i] = mux(G, x[i], y[i]).
    const int xBase = isMax ? lhsBase : rhsBase;
    const int yBase = isMax ? rhsBase : lhsBase;
    for (int i = 0; i < bw; i++) {
        for (int b : banks) emitAAP(SLOT_T1,     SLOT_DCC0,     b); // T1 = G
        for (int b : banks) emitAAP(SLOT_T0,     xBase + i,     b);
        for (int b : banks) emitAAP(SLOT_T2,     SLOT_C_0,      b);
        for (int b : banks) emitAAP(SLOT_T3,     SLOT_T0_T1_T2, b); // T3 = G&x
        for (int b : banks) emitAAP(SLOT_T1,     SLOT_DCC0N,    b); // T1 = ~G
        for (int b : banks) emitAAP(SLOT_T0,     yBase + i,     b);
        for (int b : banks) emitAAP(SLOT_T2,     SLOT_C_0,      b);
        for (int b : banks) emitAP (SLOT_T0_T1_T2,              b); // = ~G&y
        for (int b : banks) emitAAP(SLOT_T2,     SLOT_C_1,      b);
        for (int b : banks) emitAAP(outBase + i, SLOT_T1_T2_T3, b);
    }
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
// bounds and crash.  Model the inter-channel transfer as one full-row
// data-bus stream per side: ROW_RD_STREAM on the source channel (ACT +
// tRCD + columnsPerRowBuffer bursts out + tRTP + PRE) and ROW_WR_STREAM
// on the destination channel (same, with tWR recovery).  Each channel's
// data bus carries the row exactly once and the two sides pipeline like a
// host-buffered DMA (host interconnect bandwidth is assumed to not be the
// bottleneck).  This replaces the old 2-bare-ROWAP approximation, which
// charged no data movement at all and made a cross-channel copy cheaper
// than an intra-subarray RowClone FPM copy.
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
        for (int j = 0; j < bw; j++) {
            emitRdStream(outBase + j, src_bank);  // src channel: row out
            emitWrStream(lhsBase + j, dst_bank);  // dst channel: row in
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
        } else if (nr.kind == OP_SUBI) { // N - N -> N, like OP_ADDI
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_rhs = std::max(max_rhs, nr.rhs_bw);
            max_out = std::max(max_out, std::min(nr.lhs_bw, nr.rhs_bw));
        } else if (nr.kind == OP_ADDI_WIDE) { // N + N -> N+1 (carry-out row)
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_rhs = std::max(max_rhs, nr.rhs_bw);
            max_out = std::max(max_out, std::max(nr.lhs_bw, nr.rhs_bw) + 1);
        } else if (nr.kind == OP_RELU) { // unary: lhs[] -> out[]
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_out = std::max(max_out, nr.lhs_bw);
        } else if (nr.kind == OP_XNOR) { // N x N -> N, elementwise
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_rhs = std::max(max_rhs, nr.rhs_bw);
            max_out = std::max(max_out, nr.lhs_bw);
        } else if (nr.kind == OP_RANGE_SCAN) { // L bit planes -> 1 mask row
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_out = std::max(max_out, 1);
        } else if (nr.kind == OP_MIN || nr.kind == OP_MAX) { // N x N -> N
            max_lhs = std::max(max_lhs, nr.lhs_bw);
            max_rhs = std::max(max_rhs, nr.rhs_bw);
            max_out = std::max(max_out, nr.lhs_bw);
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

    const char* backendName = backend == BK_FCDRAM ? "fcdram"
                            : backend == BK_PRADA  ? "prada"
                            : "simdram";
    inform("RowOpTracePlayer: %llu records, format=%d-bank, backend=%s, "
           "slots=%d (rows/bank)",
           (unsigned long long)hdr.num_records, total_banks,
           backendName, totalSlots);

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

        // Single-bank opt: an ADDI/MULI record runs the SAME row-op on the
        // SAME rows across every bank in its mask (SIMD), which real PuD
        // hardware issues as ONE all-bank broadcast command per channel --
        // one bank's op-time, not one per bank.  Keep a single representative
        // bank per channel so bank count no longer inflates the makespan
        // (channels are independent DRAMCtrls and stay parallel).  Same name
        // and semantics as OptiPIM's single_bank_opt.  `banks` is ascending,
        // so banks of one channel are contiguous and we can dedup channels
        // with a running last-channel id.
        std::vector<int> collapsed;
        if (singleBank) {
            int lastCh = -1;
            for (int b : banks) {
                int ch = b / banksPerChannel;
                if (ch != lastCh) { collapsed.push_back(b); lastCh = ch; }
            }
        }
        const std::vector<int>& compBanks = singleBank ? collapsed : banks;

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
          case OP_RELU:
            expandRelu(r.lhs_bw, compBanks);
            break;
          case OP_SUBI:
            expandSub(r.lhs_bw, r.rhs_bw, compBanks);
            break;
          case OP_ADDI_WIDE:
            expandAddWide(r.lhs_bw, compBanks);
            break;
          case OP_XNOR:
            expandXnor(r.lhs_bw, compBanks);
            break;
          case OP_RANGE_SCAN:
            expandRangeScan(r.lhs_bw, compBanks);
            break;
          case OP_MIN:
            expandMinMax(r.lhs_bw, compBanks, /*isMax=*/false);
            break;
          case OP_MAX:
            expandMinMax(r.lhs_bw, compBanks, /*isMax=*/true);
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

    if (pendingOps.empty())
        return;

    if (perChannel) {
        if (chanPorts.empty())
            panic("RowOpTracePlayer: per_channel set but no chan_port "
                  "connected");
        partitionGroup(0);
        schedule(&chanSendEvent, curTick());
    } else {
        schedule(&sendEvent, curTick());
    }
}

// ---------------------------------------------------------------------------
// sendNextOp: issue (pump) all remaining row-ops of the current start-group,
// concurrently, until the memory back-pressures.  Ops of one start-group are
// independent (the scheduler placed them at the same time), so they are all in
// flight at once; the memory's per-bank/channel timing overlaps independent
// banks and serialises same-bank chains.  The barrier that gates the *next*
// group lives in recvTimingResp.
// ---------------------------------------------------------------------------

PacketPtr
RowOpTracePlayer::makeOpPacket(size_t idx)
{
    const PendingOp& op = pendingOps[idx];
    Addr dest = slotAddr(op.dest_slot, op.dest_bank);
    Addr src1 = slotAddr(op.src1_slot, op.src1_bank);
    Addr src2 = (op.op == Request::ROWCOPY) ? 0
                                             : slotAddr(op.src2_slot, op.src2_bank);
    return makeRowOpPacket(op.op, dest, src1, src2);
}

void
RowOpTracePlayer::sendNextOp()
{
    size_t groupEnd = groupEnds[curGroup];

    while (issueIdx < groupEnd) {
        PacketPtr pkt = makeOpPacket(issueIdx);

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

// ===========================================================================
// Per-channel issue engine (perChannel == true)
//
// The start-group barrier is preserved: only the current start-group's ops are
// in flight.  Within that group each channel drains its own sub-stream in
// order through its own port and retry slot, so back-pressure on one channel
// no longer stalls issues to the others (removes the single-port head-of-line
// blocking).  Per-channel order is kept (same-bank chains and same-channel
// cross-group dependencies stay correct); only within-group cross-channel
// issue order is relaxed, which independent DRAMCtrls have no dependency on.
// When the whole group has completed, the next group is partitioned and pumped.
// ===========================================================================
void
RowOpTracePlayer::partitionGroup(size_t g)
{
    size_t start = (g == 0) ? 0 : groupEnds[g - 1];
    size_t end   = groupEnds[g];
    size_t n     = chanPorts.size();

    chanQ.assign(n, std::vector<size_t>());
    chanCursor.assign(n, 0);
    chanRetry.assign(n, nullptr);   // barrier guarantees no retry is in flight

    for (size_t i = start; i < end; ++i) {
        int c = opChannel(i);
        if (c < 0 || (size_t)c >= n)
            panic("RowOpTracePlayer: op %llu maps to channel %d, out of range "
                  "[0,%llu)", (unsigned long long)i, c, (unsigned long long)n);
        chanQ[c].push_back(i);
    }
}

void
RowOpTracePlayer::pumpChannel(size_t c)
{
    while (chanRetry[c] == nullptr && chanCursor[c] < chanQ[c].size()) {
        size_t idx = chanQ[c][chanCursor[c]];
        PacketPtr pkt = makeOpPacket(idx);
        if (!chanPorts[c]->sendTimingReq(pkt)) {
            chanRetry[c] = pkt;      // this channel stalls; others continue
            numRetries++;
            return;
        }
        chanCursor[c]++;
        numPacketsSent++;
        if (!firstOpSeen) { firstOpSeen = true; firstOpTick = curTick(); }
    }
}

void
RowOpTracePlayer::chanPumpAll()
{
    for (size_t c = 0; c < chanPorts.size(); ++c)
        pumpChannel(c);
}

bool
RowOpTracePlayer::recvTimingRespChan(int ch, PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;

    completedCount++;
    lastOpTick = curTick();

    // Dependency barrier: advance to the next start-group only once every op of
    // the current group has retired (mirror of the single-port engine).
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
        curGroup++;
        partitionGroup(curGroup);
        schedule(&chanSendEvent, curTick() + 1);
    }
    return true;
}

void
RowOpTracePlayer::recvReqRetryChan(int ch)
{
    assert(chanRetry[ch] != nullptr);

    if (!chanPorts[ch]->sendTimingReq(chanRetry[ch])) {
        numRetries++;
        return;                      // still blocked on this channel
    }

    // Stashed packet accepted: count it and resume issuing this channel.
    chanRetry[ch] = nullptr;
    chanCursor[ch]++;
    numPacketsSent++;
    if (!firstOpSeen) { firstOpSeen = true; firstOpTick = curTick(); }
    pumpChannel(ch);
}

RowOpTracePlayer*
RowOpTracePlayerParams::create()
{
    return new RowOpTracePlayer(this);
}
