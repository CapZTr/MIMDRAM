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
#include "mem/hbm_ctrl.hh"
#include "sim/system.hh"

using namespace std;

// ============================================================
// Constructor
// ============================================================

HBMCtrl::HBMCtrl(const HBMCtrlParams* p)
    : MemCtrl(p),
      retryRdReqPC1(false), retryWrReqPC1(false),
      writesThisTimePC1(0), readsThisTimePC1(0), pendingRowOpsPC1(0),
      busStatePC1(READ), busBusyUntilPC1(0),
      nextReqEventPC1(*this), respondEventPC1(*this),
      pc1Int(p->dram1),
      partitionedQ(p->partitioned_q),
      rowBurstTicks(p->dram->tRCD),   // row-address command spans tRCD on bus
      colBurstTicks(p->dram->tBURST), // column command spans one burst
      cmdBusAvailPC0(0), cmdBusAvailPC1(0)
{
    fatal_if(!pc1Int, "HBMCtrl requires dram1 (PC1 DRAMInterface)\n");
}

// ============================================================
// SimObject lifecycle
// ============================================================

void
HBMCtrl::init()
{
    MemCtrl::init();

    // Wire PC1 back-reference
    pc1Int->ctrl = this;

    // Geometry for PC1 - same address range as PC0 but decoded separately
    const AddrRange& range = getAddrRange();
    uint64_t total_bytes = range.size();
    uint64_t rows = total_bytes /
                    (pc1Int->rowBufferSize * pc1Int->banksPerRank *
                     pc1Int->ranksPerChannel);
    pc1Int->rowsPerBank = (uint32_t)rows;
    pc1Int->columnsPerStripe = dram->columnsPerStripe; // same interleaving

    for (auto r : pc1Int->ranks)
        r->startup(curTick() + pc1Int->tREFI - pc1Int->tRP);
}

void
HBMCtrl::startup()
{
    MemCtrl::startup();
    pc1Int->timeStampOffset = divCeil(curTick(), pc1Int->tCK);
    busBusyUntilPC1 = curTick();
}

void
HBMCtrl::drainResume()
{
    MemCtrl::drainResume();
    if (!isTimingMode && system()->isTimingMode()) {
        pc1Int->startup(curTick() + pc1Int->tREFI - pc1Int->tRP);
    } else if (isTimingMode && !system()->isTimingMode()) {
        pc1Int->suspend();
    }
}

// ============================================================
// Shared command bus verification
// ============================================================

Tick
HBMCtrl::verifySingleCmd(Tick cmd_tick, bool is_pc0)
{
    // A single-cycle row command occupies the bus for rowBurstTicks.
    // Ensure it doesn't overlap with a pending command on the other PC.
    if (is_pc0) {
        Tick adj = std::max(cmd_tick, cmdBusAvailPC1);
        cmdBusAvailPC0 = adj + rowBurstTicks;
        return adj;
    } else {
        Tick adj = std::max(cmd_tick, cmdBusAvailPC0);
        cmdBusAvailPC1 = adj + rowBurstTicks;
        return adj;
    }
}

Tick
HBMCtrl::verifyMultiCmd(Tick cmd_tick, Tick burst_ticks, bool is_pc0)
{
    if (is_pc0) {
        Tick adj = std::max(cmd_tick, cmdBusAvailPC1);
        cmdBusAvailPC0 = adj + burst_ticks;
        return adj;
    } else {
        Tick adj = std::max(cmd_tick, cmdBusAvailPC0);
        cmdBusAvailPC1 = adj + burst_ticks;
        return adj;
    }
}

// ============================================================
// recvTimingReq - route by address bit 6
// ============================================================

bool
HBMCtrl::recvTimingReq(PacketPtr pkt)
{
    // Bit 6 of the physical address selects the pseudo-channel.
    // PC0 = bit 6 clear, PC1 = bit 6 set.
    bool is_pc1 = partitionedQ && ((pkt->getAddr() >> 6) & 1);

    if (!is_pc1) {
        // Delegate to MemCtrl base (handles PC0 queues)
        return MemCtrl::recvTimingReq(pkt);
    }

    // ---- PC1 path ----

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

    unsigned int size      = pkt->getSize();
    unsigned int burst     = pc1Int->burstSize;
    unsigned int pkt_count = divCeil(size, burst);

    bool goes_to_write = is_write || is_row_op;

    if (is_read && readQueueFullPC1(pkt_count)) {
        retryRdReqPC1 = true;
        numRdRetry++;
        return false;
    }
    if (goes_to_write && writeQueueFullPC1(pkt_count)) {
        retryWrReqPC1 = true;
        numWrRetry++;
        return false;
    }

    totGap += curTick() - prevArrival;
    prevArrival = curTick();

    if (is_read)
        addToReadQueuePC1(pkt, pkt_count);
    else
        addToWriteQueuePC1(pkt, pkt_count);

    if (!nextReqEventPC1.scheduled())
        schedule(nextReqEventPC1, std::max(busBusyUntilPC1, curTick()));

    return true;
}

// ============================================================
// PC1 queue helpers
// ============================================================

bool
HBMCtrl::readQueueFullPC1(unsigned int pkt_count) const
{
    return readQueuePC1.size() + pkt_count > readBufferSize;
}

bool
HBMCtrl::writeQueueFullPC1(unsigned int pkt_count) const
{
    return writeQueuePC1.size() + pkt_count > writeBufferSize;
}

void
HBMCtrl::addToReadQueuePC1(PacketPtr pkt, unsigned int pkt_count)
{
    if (pkt_count > 1) {
        BurstHelper* bh = new BurstHelper(pkt_count);
        Addr addr = pkt->getAddr();
        for (unsigned int cnt = 0; cnt < pkt_count; ++cnt, addr += pc1Int->burstSize) {
            Addr aligned = addr & ~(Addr)(pc1Int->burstSize - 1);
            MemPacket* mp = pc1Int->decodePacket(pkt, aligned, pc1Int->burstSize, true);
            mp->burstHelper = bh;
            readQueuePC1.push_back(mp);
        }
    } else {
        Addr aligned = pkt->getAddr() & ~(Addr)(pc1Int->burstSize - 1);
        MemPacket* mp = pc1Int->decodePacket(pkt, aligned, pkt->getSize(), true);
        readQueuePC1.push_back(mp);
    }
    readReqs++;
    readBursts  += pkt_count;
    bytesReadSys += pkt->getSize();
}

void
HBMCtrl::addToWriteQueuePC1(PacketPtr pkt, unsigned int pkt_count)
{
    bool is_row_op = pkt->req->isRowOp();

    if (is_row_op) {
        const Request::RowOpPayload& rop = *pkt->getPtr<Request::RowOpPayload>();
        Addr dest_addr = rop.dest;
        MemPacket* mp = pc1Int->decodePacket(pkt, dest_addr, pc1Int->burstSize, false);
        mp->is_row_op = true;
        mp->row_op    = rop.op;
        if (rop.src1 != 0) {
            MemPacket* s = pc1Int->decodePacket(pkt, rop.src1, pc1Int->burstSize, false);
            mp->src1_row = s->row; mp->src_rank = s->rank; mp->src_bank = s->bank;
            delete s;
        }
        if (rop.src2 != 0) {
            MemPacket* s = pc1Int->decodePacket(pkt, rop.src2, pc1Int->burstSize, false);
            mp->src2_row = s->row;
            delete s;
        }
        writeQueuePC1.push_back(mp);
        pendingRowOpsPC1++;
        numRowOps++;
    } else {
        Addr aligned = pkt->getAddr() & ~(Addr)(pc1Int->burstSize - 1);
        for (unsigned int cnt = 0; cnt < pkt_count; ++cnt, aligned += pc1Int->burstSize) {
            MemPacket* mp = pc1Int->decodePacket(pkt, aligned, pc1Int->burstSize, false);
            writeQueuePC1.push_back(mp);
        }
    }
    writeReqs++;
    writeBursts     += pkt_count;
    bytesWrittenSys += pkt->getSize();
}

void
HBMCtrl::accessAndRespondPC1(PacketPtr pkt, Tick static_latency)
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
// processNextReqEventPC1 - PC1 scheduler loop
// ============================================================

void
HBMCtrl::processNextReqEventPC1()
{
    if (pc1Int->allRanksBusy()) {
        schedule(nextReqEventPC1,
                 std::max(busBusyUntilPC1, curTick()) + pc1Int->tRFC);
        return;
    }
    pc1Int->checkDrainState();

    bool switch_to_write = false;

    if (busStatePC1 == WRITE) {
        if (writeQueuePC1.empty() ||
            (writeQueuePC1.size() <= (size_t)writeLowThreshold &&
             writesThisTimePC1 >= minWritesPerSwitch)) {
            busStatePC1 = READ;
            rdPerTurnAround.sample(readsThisTimePC1);
            readsThisTimePC1 = 0;
        }
    } else {
        if (!writeQueuePC1.empty()) {
            if (pendingRowOpsPC1 > 0 ||
                writeQueuePC1.size() >= (size_t)writeHighThreshold ||
                readQueuePC1.empty())
                switch_to_write = true;
        }
        if (switch_to_write) {
            busStatePC1 = WRITE;
            wrPerTurnAround.sample(writesThisTimePC1);
            writesThisTimePC1 = 0;
        }
    }

    auto& queue = (busStatePC1 == READ) ? readQueuePC1 : writeQueuePC1;
    if (queue.empty()) {
        if (!readQueuePC1.empty() || !writeQueuePC1.empty())
            schedule(nextReqEventPC1, curTick() + 1);
        return;
    }

    bool found = pc1Int->chooseNextFRFCFS(queue, 0);
    if (!found) {
        schedule(nextReqEventPC1,
                 std::max(busBusyUntilPC1, curTick()) + 1);
        return;
    }

    MemPacket* mem_pkt = queue.front();
    queue.pop_front();

    // Apply shared command bus constraints
    Tick cmd_at = verifyMultiCmd(std::max(busBusyUntilPC1, curTick()),
                                 colBurstTicks, /*is_pc0=*/false);

    Tick ready_time = pc1Int->doBurstAccess(mem_pkt, cmd_at,
                                             readQueuePC1, writeQueuePC1);
    busBusyUntilPC1 = ready_time;

    if (busStatePC1 == READ) {
        assert(mem_pkt->isRead);
        mem_pkt->readyTime = ready_time;
        respQueuePC1.push_back(mem_pkt);
        readsThisTimePC1++;

        if (!respondEventPC1.scheduled())
            schedule(respondEventPC1, respQueuePC1.front()->readyTime);

        totBusLat  += pc1Int->tBURST;
        totBankLat += cmd_at - mem_pkt->entryTime;
        totQLat    += cmd_at - mem_pkt->entryTime;
        bytesReadDRAM += pc1Int->burstSize;
        numRdTransactions++;
    } else {
        if (mem_pkt->is_row_op) {
            assert(pendingRowOpsPC1 > 0);
            pendingRowOpsPC1--;
            accessAndRespondPC1(mem_pkt->pkt, staticFrontendLatency);
        } else {
            accessAndRespondPC1(mem_pkt->pkt, staticFrontendLatency);
        }
        delete mem_pkt;
        writesThisTimePC1++;
        bytesWritten += pc1Int->burstSize;
        numWrTransactions++;
    }

    if (retryRdReqPC1 && !readQueueFullPC1(1)) {
        retryRdReqPC1 = false;
        port.sendRetryReq();
    }
    if (retryWrReqPC1 && !writeQueueFullPC1(1)) {
        retryWrReqPC1 = false;
        port.sendRetryReq();
    }

    if (!readQueuePC1.empty() || !writeQueuePC1.empty())
        schedule(nextReqEventPC1,
                 std::max(busBusyUntilPC1, curTick()));
}

// ============================================================
// processRespondEventPC1 - return PC1 reads to CPU
// ============================================================

void
HBMCtrl::processRespondEventPC1()
{
    assert(!respQueuePC1.empty());
    MemPacket* mem_pkt = respQueuePC1.front();

    if (mem_pkt->burstHelper) {
        mem_pkt->burstHelper->burstsServiced++;
        if (mem_pkt->burstHelper->burstsServiced ==
            mem_pkt->burstHelper->burstCount) {
            accessAndRespondPC1(mem_pkt->pkt,
                                staticFrontendLatency + staticBackendLatency);
            delete mem_pkt->burstHelper;
        }
    } else {
        accessAndRespondPC1(mem_pkt->pkt,
                            staticFrontendLatency + staticBackendLatency);
    }

    totMemAccLat += mem_pkt->readyTime - mem_pkt->entryTime;
    respQueuePC1.pop_front();
    delete mem_pkt;

    if (!respQueuePC1.empty())
        schedule(respondEventPC1, respQueuePC1.front()->readyTime);
}

HBMCtrl*
HBMCtrlParams::create()
{
    return new HBMCtrl(this);
}
