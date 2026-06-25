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

/**
 * @file
 * MemCtrl: FR-FCFS memory scheduler decoupled from the DRAM device model.
 *
 * MemCtrl owns the read/write queues and the bus state machine.
 * It delegates all device-level timing and bank state to DRAMInterface.
 * HBMCtrl extends MemCtrl to support dual pseudo-channels.
 */

#ifndef __MEM_CTRL_HH__
#define __MEM_CTRL_HH__

#include <deque>
#include <string>
#include <unordered_set>
#include <utility>

#include "base/statistics.hh"
#include "enums/MemSched.hh"
#include "mem/abstract_mem.hh"
#include "mem/mem_interface.hh"
#include "mem/qport.hh"
#include "params/MemCtrl.hh"
#include "sim/eventq.hh"

// Forward declaration
class DRAMInterface;

/**
 * MemCtrl is a memory controller for a single DRAM channel/pseudo-channel.
 * It implements FR-FCFS scheduling over independent read and write queues,
 * a read-write bus state machine with configurable thresholds, and optional
 * row-op (MIMDRAM/PUD) handling.
 *
 * HBMCtrl subclasses MemCtrl to manage two pseudo-channels.
 */
class MemCtrl : public AbstractMemory
{
  protected:

    // ----------------------------------------------------------------
    // Slave port - connects to the CPU-side interconnect
    // ----------------------------------------------------------------

    class MemoryPort : public QueuedSlavePort
    {
      private:
        RespPacketQueue queue;
        MemCtrl& ctrl;

      public:
        MemoryPort(const std::string& name, MemCtrl& _ctrl);

      protected:
        Tick recvAtomic(PacketPtr pkt) M5_ATTR_OVERRIDE;
        void recvFunctional(PacketPtr pkt) M5_ATTR_OVERRIDE;
        bool recvTimingReq(PacketPtr pkt) M5_ATTR_OVERRIDE;
        AddrRangeList getAddrRanges() const M5_ATTR_OVERRIDE;
    };

    MemoryPort port;

    // ----------------------------------------------------------------
    // Bus state
    // ----------------------------------------------------------------

    enum BusState { READ, WRITE };
    BusState busState;
    Tick     busBusyUntil;

    // ----------------------------------------------------------------
    // Queues
    // ----------------------------------------------------------------

    // Per-packet tracking set to avoid duplicates in the write queue
    std::unordered_set<Addr> isInWriteQueue;

    std::deque<MemPacket*> readQueue;
    std::deque<MemPacket*> writeQueue;
    std::deque<MemPacket*> respQueue;  // read responses ready to return

    // ----------------------------------------------------------------
    // Scheduling params
    // ----------------------------------------------------------------

    const unsigned int readBufferSize;
    const unsigned int writeBufferSize;

    const double writeHighThreshold;
    const double writeLowThreshold;
    const unsigned int minWritesPerSwitch;

    const Enums::MemSched memSchedPolicy;

    const Tick staticFrontendLatency;
    const Tick staticBackendLatency;

    // ----------------------------------------------------------------
    // Counters that drive write-drain logic
    // ----------------------------------------------------------------

    unsigned int writesThisTime;
    unsigned int readsThisTime;

    // Row ops pending (written into writeQueue; defer write drain until done)
    unsigned int pendingRowOps;

    // Whether we're in timing mode (cached from system()->isTimingMode())
    bool isTimingMode;

    // ----------------------------------------------------------------
    // DRAM device interface
    // ----------------------------------------------------------------

    DRAMInterface* dram;

    // ----------------------------------------------------------------
    // Events
    // ----------------------------------------------------------------

    void processNextReqEvent();
    EventWrapper<MemCtrl, &MemCtrl::processNextReqEvent> nextReqEvent;

    void processRespondEvent();
    EventWrapper<MemCtrl, &MemCtrl::processRespondEvent> respondEvent;

    // ----------------------------------------------------------------
    // Internal helpers
    // ----------------------------------------------------------------

    bool readQueueFull(unsigned int pkt_count) const;
    bool writeQueueFull(unsigned int pkt_count) const;

    void addToReadQueue(PacketPtr pkt, unsigned int pkt_count);
    void addToWriteQueue(PacketPtr pkt, unsigned int pkt_count);

    void accessAndRespond(PacketPtr pkt, Tick static_latency);

    bool chooseNext(std::deque<MemPacket*>& queue,
                    Tick extra_col_delay) const;

    std::pair<MemPacket*, bool>
    chooseNextFRFCFS(std::deque<MemPacket*>& queue,
                     Tick extra_col_delay) const;

    // ----------------------------------------------------------------
    // Statistics
    // ----------------------------------------------------------------

    Stats::Scalar readReqs;
    Stats::Scalar writeReqs;
    Stats::Scalar readBursts;
    Stats::Scalar writeBursts;
    Stats::Scalar bytesReadSys;
    Stats::Scalar bytesWrittenSys;
    Stats::Scalar servicedByWrQ;
    Stats::Scalar mergedWrBursts;
    Stats::Scalar neitherReadNorWrite;
    Stats::Vector perBankRdBursts;
    Stats::Vector perBankWrBursts;
    Stats::Scalar numRdRetry;
    Stats::Scalar numWrRetry;
    Stats::Scalar totGap;
    Stats::Vector readPktSize;
    Stats::Vector writePktSize;
    Stats::Vector rdQLenPdf;
    Stats::Vector wrQLenPdf;
    Stats::Histogram rdPerTurnAround;
    Stats::Histogram wrPerTurnAround;
    Stats::Scalar bytesReadDRAM;
    Stats::Scalar bytesWritten;
    Stats::Scalar bytesReadWrQ;
    Stats::Scalar numRdTransactions;
    Stats::Scalar numWrTransactions;
    Stats::Scalar totQLat;
    Stats::Scalar totMemAccLat;
    Stats::Scalar totBusLat;
    Stats::Scalar totBankLat;
    Stats::Formula avgQLat;
    Stats::Formula avgMemAccLat;
    Stats::Formula avgBusLat;
    Stats::Formula avgBankLat;
    Stats::Scalar numRowOps;
    Stats::Scalar numPendingRowOps;

    bool retryRdReq;
    bool retryWrReq;

    Tick prevArrival;
    Tick lastReqTime;

    // ----------------------------------------------------------------
    // MIMDRAM helpers
    // ----------------------------------------------------------------

    /**
     * Encode a row-op destination / source address into the decoded
     * row field of a MemPacket.  Returns false if the op spans ranks or
     * banks in a way that can't be served atomically (falls back to the
     * normal write path for those).
     */
    bool decodeRowOpAddresses(MemPacket* mem_pkt, PacketPtr pkt);

    // pudComputeN is forwarded to dram->pudComputeN().

  public:

    // Track which rank is currently being served (used by refresh drain).
    // Needs to be public so DRAMInterface::Rank can read it.
    uint8_t activeRank;

    MemCtrl(const MemCtrlParams* p);

    // ----------------------------------------------------------------
    // SimObject interface
    // ----------------------------------------------------------------

    DrainState drain() M5_ATTR_OVERRIDE;
    virtual void drainResume() M5_ATTR_OVERRIDE;
    void init() M5_ATTR_OVERRIDE;
    void startup() M5_ATTR_OVERRIDE;
    void regStats() M5_ATTR_OVERRIDE;

    BaseSlavePort& getSlavePort(const std::string& if_name,
                                PortID idx = InvalidPortID) M5_ATTR_OVERRIDE;

    // ----------------------------------------------------------------
    // Timing/atomic request entry points (virtual so HBMCtrl can override)
    // ----------------------------------------------------------------

    virtual bool recvTimingReq(PacketPtr pkt);
    virtual Tick recvAtomic(PacketPtr pkt);

    // ----------------------------------------------------------------
    // API used by DRAMInterface::Rank::processPowerEvent
    // ----------------------------------------------------------------

    void scheduleNextReq(Tick tick);
    bool nextReqEventScheduled() const { return nextReqEvent.scheduled(); }
};

#endif // __MEM_CTRL_HH__
