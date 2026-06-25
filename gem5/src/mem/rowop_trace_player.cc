#include "mem/rowop_trace_player.hh"

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

RowOpTracePlayer::RowOpTracePlayer(const RowOpTracePlayerParams* p)
    : MemObject(p),
      port("port", *this),
      traceFile(p->trace_file),
      baseAddr(p->base_addr),
      banksPerChannel(p->banks_per_channel),
      channelSize(p->channel_size),
      rowStride(p->row_stride),
      masterID(p->system->getMasterId(name())),
      currentOp(0),
      waitingResp(false),
      retryPkt(nullptr),
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
// execute_add  (mirrors C code in 93_simdram_schedule_runner.c)
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandAdd(int lhs_bw, int rhs_bw,
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
// execute_mul  (mirrors C code in 93_simdram_schedule_runner.c)
// ---------------------------------------------------------------------------
void
RowOpTracePlayer::expandMul(int lhs_bw, int rhs_bw,
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

    inform("RowOpTracePlayer: %llu records, format=%d-bank, slots=%d (rows/bank)",
           (unsigned long long)hdr.num_records, total_banks, totalSlots);

    if (totalSlots >= 512)
        panic("RowOpTracePlayer: %d slots exceed rows_per_subarray (512)", totalSlots);

    // --- Expand records into PendingOps ---
    for (uint64_t i = 0; i < hdr.num_records; i++) {
        const NormRecord& r = records[i];

        // Build global bank list from 128-bit bitmask.
        std::vector<int> banks;
        for (int b = 0; b < total_banks; b++)
            if (r.banks[b >> 6] & (1ull << (b & 63)))
                banks.push_back(b);

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
// sendNextOp: convert current PendingOp to a Packet and send it
// ---------------------------------------------------------------------------

void
RowOpTracePlayer::sendNextOp()
{
    assert(!waitingResp);
    assert(currentOp < pendingOps.size());

    const PendingOp& op = pendingOps[currentOp];

    Addr dest = slotAddr(op.dest_slot, op.dest_bank);
    Addr src1 = slotAddr(op.src1_slot, op.src1_bank);
    Addr src2 = (op.op == Request::ROWCOPY) ? 0
                                             : slotAddr(op.src2_slot, op.src2_bank);

    PacketPtr pkt = makeRowOpPacket(op.op, dest, src1, src2);

    DPRINTF(RowOpTracePlayer,
            "Sending op %llu/%llu: op=%d dest=0x%llx src1=0x%llx src2=0x%llx\n",
            (unsigned long long)currentOp, (unsigned long long)pendingOps.size(),
            (int)op.op, (unsigned long long)dest,
            (unsigned long long)src1, (unsigned long long)src2);

    if (!port.sendTimingReq(pkt)) {
        // Back-pressure: stash and wait for retry
        retryPkt = pkt;
        numRetries++;
    } else {
        waitingResp = true;
        numPacketsSent++;
    }
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
    assert(waitingResp);

    // Clean up the packet
    delete pkt->req;
    delete pkt;

    waitingResp = false;
    currentOp++;

    if (currentOp >= pendingOps.size()) {
        inform("RowOpTracePlayer: all %llu packets completed, exiting",
               (unsigned long long)pendingOps.size());
        exitSimLoop("RowOpTracePlayer: trace replay complete");
        return true;
    }

    // Schedule the next send at the next tick
    schedule(&sendEvent, curTick() + 1);
    return true;
}

void
RowOpTracePlayer::recvReqRetry()
{
    assert(retryPkt != nullptr);

    PacketPtr pkt = retryPkt;
    retryPkt = nullptr;

    if (!port.sendTimingReq(pkt)) {
        retryPkt = pkt;
        numRetries++;
    } else {
        waitingResp = true;
        numPacketsSent++;
    }
}

RowOpTracePlayer*
RowOpTracePlayerParams::create()
{
    return new RowOpTracePlayer(this);
}
