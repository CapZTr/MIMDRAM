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
 * HBMCtrl: dual pseudo-channel HBM memory controller.
 *
 * Extends MemCtrl (which manages pseudo-channel 0) to add a second
 * pseudo-channel (PC1).  The two pseudo-channels share a command bus but
 * have independent data buses and independent bank/rank state (each modelled
 * by its own DRAMInterface).
 *
 * Address routing: bit 6 of the physical address selects PC0 (0) or PC1 (1).
 *
 * When partitioned_q is false both PCs share the read/write queues in MemCtrl
 * (simple/legacy behaviour).  When partitioned_q is true each PC gets its own
 * independent queues and scheduler loop.
 */

#ifndef __MEM_HBM_CTRL_HH__
#define __MEM_HBM_CTRL_HH__

#include <deque>
#include <string>

#include "mem/mem_ctrl.hh"
#include "params/HBMCtrl.hh"

// Forward declaration
class DRAMInterface;

/**
 * HBMCtrl manages two HBM pseudo-channels (PC0 and PC1) that share one
 * physical command bus.  MemCtrl manages PC0; HBMCtrl adds the PC1 path.
 */
class HBMCtrl : public MemCtrl
{
  private:

    // ----------------------------------------------------------------
    // PC1-specific queues and state (PC0 state lives in MemCtrl base)
    // ----------------------------------------------------------------

    std::deque<MemPacket*> readQueuePC1;
    std::deque<MemPacket*> writeQueuePC1;
    std::deque<MemPacket*> respQueuePC1;

    bool retryRdReqPC1;
    bool retryWrReqPC1;

    unsigned int writesThisTimePC1;
    unsigned int readsThisTimePC1;
    unsigned int pendingRowOpsPC1;

    BusState busStatePC1;
    Tick     busBusyUntilPC1;

    // ----------------------------------------------------------------
    // PC1 scheduler events
    // ----------------------------------------------------------------

    void processNextReqEventPC1();
    EventWrapper<HBMCtrl, &HBMCtrl::processNextReqEventPC1> nextReqEventPC1;

    void processRespondEventPC1();
    EventWrapper<HBMCtrl, &HBMCtrl::processRespondEventPC1> respondEventPC1;

    // ----------------------------------------------------------------
    // Second DRAM interface (PC1)
    // ----------------------------------------------------------------

    DRAMInterface* pc1Int;

    // Whether to use independent per-PC queues
    const bool partitionedQ;

    // ----------------------------------------------------------------
    // Shared command bus: burst durations for verifySingleCmd / verifyMultiCmd
    // ----------------------------------------------------------------

    // tBURST for row-address (ACT/PRE) commands - shared bus constraint
    const Tick rowBurstTicks;
    // tBURST for column (RD/WR) commands - shared bus constraint
    const Tick colBurstTicks;

    // Tracks when the shared command bus is free for each PC
    Tick cmdBusAvailPC0;
    Tick cmdBusAvailPC1;

    // ----------------------------------------------------------------
    // Shared-command-bus verification
    // ----------------------------------------------------------------

    /**
     * Check that a single-cycle command (ACT, PRE, REF) issued to one PC
     * does not collide with a pending command on the other PC.
     * @param cmd_tick   Proposed tick for the command.
     * @param is_pc0     True if cmd is for PC0.
     * @return Adjusted tick that satisfies the shared-bus constraint.
     */
    Tick verifySingleCmd(Tick cmd_tick, bool is_pc0);

    /**
     * Check that a multi-cycle column command (RD, WR) fits in the shared
     * command bus without overlapping with the other PC's commands.
     * @param cmd_tick       Proposed tick for the column command.
     * @param burst_ticks    Duration of the column command on the bus.
     * @param is_pc0         True if cmd is for PC0.
     * @return Adjusted tick that satisfies the shared-bus constraint.
     */
    Tick verifyMultiCmd(Tick cmd_tick, Tick burst_ticks, bool is_pc0);

    // ----------------------------------------------------------------
    // Per-PC queue helpers
    // ----------------------------------------------------------------

    bool readQueueFullPC1(unsigned int pkt_count) const;
    bool writeQueueFullPC1(unsigned int pkt_count) const;

    void addToReadQueuePC1(PacketPtr pkt, unsigned int pkt_count);
    void addToWriteQueuePC1(PacketPtr pkt, unsigned int pkt_count);

    void accessAndRespondPC1(PacketPtr pkt, Tick static_latency);

  public:

    HBMCtrl(const HBMCtrlParams* p);

    // ----------------------------------------------------------------
    // SimObject overrides
    // ----------------------------------------------------------------

    void init()        M5_ATTR_OVERRIDE;
    void startup()     M5_ATTR_OVERRIDE;
    void drainResume() M5_ATTR_OVERRIDE;

    bool recvTimingReq(PacketPtr pkt) M5_ATTR_OVERRIDE;
};

#endif // __MEM_HBM_CTRL_HH__
