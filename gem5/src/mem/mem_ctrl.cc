/*
 * Copyright (c) 2012-2015 ARM Limited
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/Drain.hh"
#include "mem/dram_interface.hh"
#include "mem/mem_ctrl.hh"
#include "sim/system.hh"

using namespace std;

// ============================================================
// Constructor
// ============================================================

MemCtrl::MemCtrl(const MemCtrlParams* p)
    : AbstractMemory(p),
      port(name() + ".port", *this),
      busState(READ), busBusyUntil(0),
      readBufferSize(p->read_buffer_size),
      writeBufferSize(p->write_buffer_size),
      writeHighThreshold(writeBufferSize * p->write_high_thresh_perc / 100.0),
      writeLowThreshold(writeBufferSize * p->write_low_thresh_perc / 100.0),
      minWritesPerSwitch(p->min_writes_per_switch),
      memSchedPolicy(p->mem_sched_policy),
      staticFrontendLatency(p->static_frontend_latency),
      staticBackendLatency(p->static_backend_latency),
      writesThisTime(0), readsThisTime(0),
      pendingRowOps(0), activeRank(0), isTimingMode(false),
      dram(p->dram),
      nextReqEvent(*this), respondEvent(*this),
      retryRdReq(false), retryWrReq(false),
      prevArrival(0), lastReqTime(0)
{
    fatal_if(!dram, "MemCtrl requires a DRAMInterface\n");
}

// ============================================================
// Port
// ============================================================

MemCtrl::MemoryPort::MemoryPort(const string& name, MemCtrl& _ctrl)
    : QueuedSlavePort(name, &_ctrl, queue), queue(_ctrl, *this),
      ctrl(_ctrl)
{ }

AddrRangeList
MemCtrl::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(ctrl.getAddrRange());
    return ranges;
}

Tick
MemCtrl::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return ctrl.recvAtomic(pkt);
}

void
MemCtrl::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(ctrl.name());
    if (!queue.checkFunctional(pkt))
        ctrl.functionalAccess(pkt);
    pkt->popLabel();
}

bool
MemCtrl::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    return ctrl.recvTimingReq(pkt);
}

BaseSlavePort&
MemCtrl::getSlavePort(const string& if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    return MemObject::getSlavePort(if_name, idx);
}

// ============================================================
// SimObject lifecycle
// ============================================================

void
MemCtrl::init()
{
    AbstractMemory::init();

    if (!port.isConnected())
        fatal("MemCtrl %s is unconnected\n", name());

    // Wire back-reference from DRAMInterface to us
    dram->ctrl = this;

    // Compute geometry
    const AddrRange& range = getAddrRange();
    uint64_t total_bytes = range.size();
    uint64_t rows = total_bytes /
                    (dram->rowBufferSize * dram->banksPerRank *
                     dram->ranksPerChannel);
    dram->rowsPerBank = (uint32_t)rows;
    dram->columnsPerStripe = (uint32_t)(range.interleaved()
                                ? range.granularity() / dram->burstSize
                                : 1);

    DPRINTF(DRAM, "MemCtrl: %llu bytes, %u rows per bank\n",
            total_bytes, dram->rowsPerBank);

    isTimingMode = system()->isTimingMode();

    if (isTimingMode) {
        for (auto r : dram->ranks)
            r->startup(curTick() + dram->tREFI - dram->tRP);
    }
}

void
MemCtrl::startup()
{
    dram->timeStampOffset = divCeil(curTick(), dram->tCK);
    busBusyUntil = curTick();
    lastReqTime  = curTick();
}

// ============================================================
// Drain
// ============================================================

DrainState
MemCtrl::drain()
{
    if (readQueue.empty() && writeQueue.empty() && respQueue.empty() &&
        !nextReqEvent.scheduled()) {
        DPRINTF(Drain, "MemCtrl done draining\n");
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "MemCtrl not drained\n");
        return DrainState::Draining;
    }
}

void
MemCtrl::drainResume()
{
    if (!isTimingMode && system()->isTimingMode()) {
        isTimingMode = true;
        dram->startup(curTick() + dram->tREFI - dram->tRP);
    } else if (isTimingMode && !system()->isTimingMode()) {
        isTimingMode = false;
        dram->suspend();
    }
}

// ============================================================
// Timing request path
// ============================================================

bool
MemCtrl::readQueueFull(unsigned int pkt_count) const
{
    return readQueue.size() + pkt_count > readBufferSize;
}

bool
MemCtrl::writeQueueFull(unsigned int pkt_count) const
{
    return writeQueue.size() + pkt_count > writeBufferSize;
}

bool
MemCtrl::recvTimingReq(PacketPtr pkt)
{
    assert(!(pkt->isRead() && pkt->isWrite()));

    if (pkt->memInhibitAsserted()) {
        pkt->makeTimingResponse();
        port.schedTimingResp(pkt, curTick());
        return true;
    }

    bool is_read  = pkt->isRead();
    bool is_write = pkt->isWrite();
    bool is_row_op = pkt->req->isRowOp();

    if (!is_read && !is_write && !is_row_op) {
        neitherReadNorWrite++;
        pkt->makeTimingResponse();
        port.schedTimingResp(pkt, curTick() + staticFrontendLatency);
        return true;
    }

    assert(is_read || is_write || is_row_op);
    assert(!(is_read && is_write));

    unsigned int size   = pkt->getSize();
    unsigned int burst  = dram->burstSize;
    unsigned int pkt_count = divCeil(size, burst);

    // Row ops are treated as writes (they occupy write queue slots)
    bool goes_to_write_q = is_write || is_row_op;

    if (is_read && readQueueFull(pkt_count)) {
        DPRINTF(DRAM, "Read queue full, retry\n");
        retryRdReq = true;
        numRdRetry++;
        return false;
    }

    if (goes_to_write_q && writeQueueFull(pkt_count)) {
        DPRINTF(DRAM, "Write queue full, retry\n");
        retryWrReq = true;
        numWrRetry++;
        return false;
    }

    totGap += curTick() - prevArrival;
    prevArrival = curTick();

    if (is_read)
        addToReadQueue(pkt, pkt_count);
    else
        addToWriteQueue(pkt, pkt_count);

    if (!nextReqEvent.scheduled())
        schedule(nextReqEvent, std::max(busBusyUntil, curTick()));

    return true;
}

Tick
MemCtrl::recvAtomic(PacketPtr pkt)
{
    if (pkt->memInhibitAsserted())
        return 0;

    Tick latency = 0;
    if (pkt->isRead() || pkt->isWrite()) {
        access(pkt);
        latency = staticFrontendLatency + staticBackendLatency;
    }
    return latency;
}

void
MemCtrl::addToReadQueue(PacketPtr pkt, unsigned int pkt_count)
{
    if (pkt_count > 1) {
        BurstHelper* burst_helper = new BurstHelper(pkt_count);
        Addr addr = pkt->getAddr();
        for (unsigned int cnt = 0; cnt < pkt_count; ++cnt, addr += dram->burstSize) {
            Addr aligned = addr & ~(Addr)(dram->burstSize - 1);
            MemPacket* mp = dram->decodePacket(pkt, aligned, dram->burstSize, true);
            mp->burstHelper = burst_helper;
            readQueue.push_back(mp);
        }
    } else {
        Addr aligned = pkt->getAddr() & ~(Addr)(dram->burstSize - 1);
        MemPacket* mp = dram->decodePacket(pkt, aligned, pkt->getSize(), true);
        readQueue.push_back(mp);
    }

    readReqs++;
    readBursts  += pkt_count;
    bytesReadSys += pkt->getSize();
    rdQLenPdf[std::min((unsigned)readQueue.size() - 1,
                       rdQLenPdf.size() - 1)]++;
}

void
MemCtrl::addToWriteQueue(PacketPtr pkt, unsigned int pkt_count)
{
    bool is_row_op = pkt->req->isRowOp();

    if (!is_row_op && isInWriteQueue.count(pkt->getAddr())) {
        // Merge into existing write
        mergedWrBursts++;
    } else {
        if (!is_row_op)
            isInWriteQueue.insert(pkt->getAddr());
    }

    if (is_row_op) {
        Request::RowOpPayload* rop_ptr = pkt->getPtr<Request::RowOpPayload>();
        const Request::RowOpPayload& rop = *rop_ptr;
        Addr dest_addr = rop.dest;

        MemPacket* mp = dram->decodePacket(pkt, dest_addr, dram->burstSize, false);
        mp->is_row_op = true;
        mp->row_op    = rop.op;
        mp->write_data = 0;

        // Decode src1 and src2 into row numbers within the same bank
        if (rop.src1 != 0) {
            MemPacket* src1_mp = dram->decodePacket(pkt, rop.src1, dram->burstSize, false);
            mp->src1_row  = src1_mp->row;
            mp->src_rank  = src1_mp->rank;
            mp->src_bank  = src1_mp->bank;
            delete src1_mp;
        }
        if (rop.src2 != 0) {
            MemPacket* src2_mp = dram->decodePacket(pkt, rop.src2, dram->burstSize, false);
            mp->src2_row = src2_mp->row;
            delete src2_mp;
        }

        if (rop.op == Request::BULK_WRITE) {
            // write_data is carried in the packet data
            if (pkt->hasData())
                mp->write_data = *(uint64_t*)pkt->getConstPtr<uint8_t>();
        }

        writeQueue.push_back(mp);
        pendingRowOps++;
        numRowOps++;
    } else {
        Addr aligned = pkt->getAddr() & ~(Addr)(dram->burstSize - 1);
        for (unsigned int cnt = 0; cnt < pkt_count; ++cnt,
                 aligned += dram->burstSize) {
            MemPacket* mp = dram->decodePacket(pkt, aligned, dram->burstSize, false);
            writeQueue.push_back(mp);
        }
    }

    writeReqs++;
    writeBursts   += pkt_count;
    bytesWrittenSys += pkt->getSize();
    wrQLenPdf[std::min((unsigned)writeQueue.size() - 1,
                       wrQLenPdf.size() - 1)]++;
}

// ============================================================
// processNextReqEvent - FR-FCFS scheduler main loop
// ============================================================

void
MemCtrl::processNextReqEvent()
{
    if (dram->allRanksBusy()) {
        // All ranks refreshing - come back after tRFC
        schedule(nextReqEvent, std::max(busBusyUntil, curTick()) + dram->tRFC);
        return;
    }
    dram->checkDrainState();

    bool switch_to_write = false;

    if (busState == WRITE) {
        // Continue writing until low threshold or exhausted
        if (writeQueue.empty() ||
            (writeQueue.size() <= writeLowThreshold && writesThisTime >= minWritesPerSwitch)) {
            busState = READ;
            rdPerTurnAround.sample(readsThisTime);
            readsThisTime = 0;
            DPRINTF(DRAM, "Switching to READ\n");
        }
    } else {
        // Consider switching to WRITE
        if (!writeQueue.empty()) {
            if (pendingRowOps > 0 ||
                writeQueue.size() >= (size_t)writeHighThreshold ||
                readQueue.empty()) {
                switch_to_write = true;
            }
        }
        if (switch_to_write) {
            busState = WRITE;
            wrPerTurnAround.sample(writesThisTime);
            writesThisTime = 0;
            DPRINTF(DRAM, "Switching to WRITE\n");
        }
    }

    auto& queue = (busState == READ) ? readQueue : writeQueue;
    if (queue.empty()) {
        // Nothing to do: either we just switched and the opposite queue is empty,
        // or both queues are empty (drain scenario).
        if (!readQueue.empty() || !writeQueue.empty())
            schedule(nextReqEvent, curTick() + 1);
        return;
    }

    // FR-FCFS: pick the best candidate
    Tick extra_delay = (busState == READ &&
                        busState == WRITE) ? dram->tRTW : 0;
    if (busState == WRITE && !writeQueue.empty())
        extra_delay = 0;

    bool found = dram->chooseNextFRFCFS(queue, extra_delay);
    if (!found) {
        // No rank available right now - retry later
        schedule(nextReqEvent, std::max(busBusyUntil, curTick()) + 1);
        return;
    }

    MemPacket* mem_pkt = queue.front();
    queue.pop_front();

    // Track active rank for refresh drain
    activeRank = mem_pkt->rank;

    // Issue the access
    Tick cmd_at = std::max(busBusyUntil, curTick());
    Tick ready_time = dram->doBurstAccess(mem_pkt, cmd_at, readQueue, writeQueue);

    // Advance bus-busy-until
    if (!mem_pkt->is_row_op)
        busBusyUntil = ready_time;
    else
        busBusyUntil = ready_time; // for row ops ready_time == end of last PRE

    if (busState == READ) {
        // Push to response queue
        assert(mem_pkt->isRead);
        mem_pkt->readyTime = ready_time;
        respQueue.push_back(mem_pkt);
        readsThisTime++;

        if (!respondEvent.scheduled())
            schedule(respondEvent, respQueue.front()->readyTime);

        // Stats
        totBusLat   += dram->tBURST;
        totBankLat  += cmd_at - mem_pkt->entryTime;
        totQLat     += cmd_at - mem_pkt->entryTime;
        bytesReadDRAM += dram->burstSize;
        numRdTransactions++;
    } else {
        // Write (including row op): delete the MemPacket now
        if (mem_pkt->is_row_op) {
            assert(pendingRowOps > 0);
            pendingRowOps--;
            numPendingRowOps = pendingRowOps;
            // Complete the original packet
            accessAndRespond(mem_pkt->pkt, staticFrontendLatency);
        } else {
            isInWriteQueue.erase(mem_pkt->addr);
            accessAndRespond(mem_pkt->pkt, staticFrontendLatency);
        }
        delete mem_pkt;
        writesThisTime++;
        bytesWritten += dram->burstSize;
        numWrTransactions++;
    }

    // Retry stalled requests
    if (retryRdReq && !readQueueFull(1)) {
        retryRdReq = false;
        port.sendRetryReq();
    }
    if (retryWrReq && !writeQueueFull(1)) {
        retryWrReq = false;
        port.sendRetryReq();
    }

    // Schedule next
    if (!readQueue.empty() || !writeQueue.empty()) {
        Tick next = std::max(busBusyUntil, curTick());
        schedule(nextReqEvent, next);
    }
}

// ============================================================
// processRespondEvent - return reads to CPU
// ============================================================

void
MemCtrl::processRespondEvent()
{
    assert(!respQueue.empty());
    MemPacket* mem_pkt = respQueue.front();

    if (mem_pkt->burstHelper) {
        mem_pkt->burstHelper->burstsServiced++;
        if (mem_pkt->burstHelper->burstsServiced ==
            mem_pkt->burstHelper->burstCount) {
            // Last sub-burst: respond to the original packet
            accessAndRespond(mem_pkt->pkt,
                             staticFrontendLatency + staticBackendLatency);
            delete mem_pkt->burstHelper;
        }
    } else {
        accessAndRespond(mem_pkt->pkt,
                         staticFrontendLatency + staticBackendLatency);
    }

    totMemAccLat += mem_pkt->readyTime - mem_pkt->entryTime;
    respQueue.pop_front();
    delete mem_pkt;

    if (!respQueue.empty())
        schedule(respondEvent, respQueue.front()->readyTime);
}

void
MemCtrl::accessAndRespond(PacketPtr pkt, Tick static_latency)
{
    bool needsResponse = pkt->needsResponse();
    access(pkt);
    if (needsResponse) {
        pkt->makeTimingResponse();
        port.schedTimingResp(pkt, curTick() + static_latency);
    } else {
        delete pkt;
    }
}

// ============================================================
// scheduleNextReq (called from DRAMInterface::Rank after refresh)
// ============================================================

void
MemCtrl::scheduleNextReq(Tick tick)
{
    if (!nextReqEvent.scheduled())
        schedule(nextReqEvent, tick);
    else if (nextReqEvent.when() > tick)
        reschedule(nextReqEvent, tick);
}

// ============================================================
// chooseNext helpers (delegated to DRAMInterface)
// ============================================================

bool
MemCtrl::chooseNext(std::deque<MemPacket*>& queue,
                    Tick extra_col_delay) const
{
    return dram->chooseNextFRFCFS(queue, extra_col_delay);
}

// ============================================================
// regStats
// ============================================================

void
MemCtrl::regStats()
{
    AbstractMemory::regStats();

    using namespace Stats;

    readReqs.name(name() + ".readReqs").desc("Number of read requests");
    writeReqs.name(name() + ".writeReqs").desc("Number of write requests");
    readBursts.name(name() + ".readBursts").desc("Number of DRAM read bursts");
    writeBursts.name(name() + ".writeBursts").desc("Number of DRAM write bursts");
    bytesReadSys.name(name() + ".bytesReadSys").desc("Total bytes read (system side)");
    bytesWrittenSys.name(name() + ".bytesWrittenSys").desc("Total bytes written (system side)");
    servicedByWrQ.name(name() + ".servicedByWrQ").desc("Reads satisfied by write queue");
    mergedWrBursts.name(name() + ".mergedWrBursts").desc("Write bursts merged/coalesced");
    neitherReadNorWrite.name(name() + ".neitherReadNorWrite").desc("Non-R/W transactions");

    perBankRdBursts.init(dram->banksPerRank * dram->ranksPerChannel)
        .name(name() + ".perBankRdBursts").desc("Read bursts per bank");
    perBankWrBursts.init(dram->banksPerRank * dram->ranksPerChannel)
        .name(name() + ".perBankWrBursts").desc("Write bursts per bank");

    numRdRetry.name(name() + ".numRdRetry").desc("Read retry count");
    numWrRetry.name(name() + ".numWrRetry").desc("Write retry count");
    totGap.name(name() + ".totGap").desc("Sum of gaps between requests");

    readPktSize.init(ceilLog2(dram->burstSize) + 1)
        .name(name() + ".readPktSize").desc("Read packet size histogram");
    writePktSize.init(ceilLog2(dram->burstSize) + 1)
        .name(name() + ".writePktSize").desc("Write packet size histogram");

    rdQLenPdf.init(readBufferSize)
        .name(name() + ".rdQLenPdf").desc("Read queue occupancy PDF");
    wrQLenPdf.init(writeBufferSize)
        .name(name() + ".wrQLenPdf").desc("Write queue occupancy PDF");

    rdPerTurnAround.init(readBufferSize)
        .name(name() + ".rdPerTurnAround").desc("Reads per R-W turnaround");
    wrPerTurnAround.init(writeBufferSize)
        .name(name() + ".wrPerTurnAround").desc("Writes per W-R turnaround");

    bytesReadDRAM.name(name() + ".bytesReadDRAM").desc("Bytes read from DRAM");
    bytesWritten.name(name() + ".bytesWritten").desc("Bytes written to DRAM");
    bytesReadWrQ.name(name() + ".bytesReadWrQ").desc("Bytes read from write queue");

    numRdTransactions.name(name() + ".numRdTransactions").desc("Read transactions served");
    numWrTransactions.name(name() + ".numWrTransactions").desc("Write transactions served");
    totQLat.name(name() + ".totQLat").desc("Total queue latency");
    totMemAccLat.name(name() + ".totMemAccLat").desc("Total memory access latency");
    totBusLat.name(name() + ".totBusLat").desc("Total bus latency");
    totBankLat.name(name() + ".totBankLat").desc("Total bank latency");

    avgQLat.name(name() + ".avgQLat").desc("Average queue latency")
        .precision(2);
    avgQLat = totQLat / numRdTransactions;

    avgMemAccLat.name(name() + ".avgMemAccLat").desc("Average memory access latency")
        .precision(2);
    avgMemAccLat = totMemAccLat / numRdTransactions;

    avgBusLat.name(name() + ".avgBusLat").desc("Average bus latency")
        .precision(2);
    avgBusLat = totBusLat / numRdTransactions;

    avgBankLat.name(name() + ".avgBankLat").desc("Average bank latency")
        .precision(2);
    avgBankLat = totBankLat / numRdTransactions;

    numRowOps.name(name() + ".numRowOps").desc("In-DRAM row operations issued");
    numPendingRowOps.name(name() + ".numPendingRowOps").desc("In-DRAM row ops in flight");

    dram->regStats();
}

MemCtrl*
MemCtrlParams::create()
{
    return new MemCtrl(this);
}
