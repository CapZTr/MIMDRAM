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
 * DRAMInterface: device-level DRAM model with bank/rank state machines,
 * timing enforcement, and MIMDRAM in-DRAM compute primitives.
 *
 * This class separates the device model from the scheduler (MemCtrl),
 * enabling dual pseudo-channel controllers (HBMCtrl) to instantiate two
 * independent DRAMInterface objects sharing a single MemCtrl scheduler.
 */

#ifndef __MEM_DRAM_INTERFACE_HH__
#define __MEM_DRAM_INTERFACE_HH__

#include <deque>
#include <list>
#include <string>

#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/PageManage.hh"
#include "mem/drampower.hh"
#include "mem/mem_interface.hh"
#include "params/DRAMInterface.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

// Forward declaration: MemCtrl provides the callback to restart scheduling
// after a rank finishes refresh.
class MemCtrl;

/**
 * DRAMInterface models the device-level timing of a single DRAM channel
 * (or pseudo-channel).  It owns the bank and rank state machines, enforces
 * DRAM timing constraints, and provides MIMDRAM row-operation primitives.
 *
 * MemCtrl (the scheduler) holds a pointer to one or two DRAMInterface
 * objects and calls doBurstAccess() / chooseNextFRFCFS() on them.
 */
class DRAMInterface : public SimObject
{
  public:

    // ----------------------------------------------------------------
    // Bank class
    // ----------------------------------------------------------------

    /**
     * Tracks per-bank state: open row, timing constraints, and access counters.
     * The large collection of negative sentinel values encode MIMDRAM
     * intermediate row identifiers used during in-DRAM computation sequences.
     */
    class Bank
    {
      public:
        static const uint32_t NO_ROW      = -1;
        static const uint32_t DOUBLE_ROW  = -2;

        // MIMDRAM intermediate-row sentinels (used in aapBank / apBank sequences)
        static const uint32_t B_T0         =  -3;
        static const uint32_t B_T1         =  -4;
        static const uint32_t B_T2         =  -5;
        static const uint32_t B_T3         =  -6;
        static const uint32_t B_DCC0       =  -7;
        static const uint32_t B_DCC0N      =  -8;
        static const uint32_t B_DCC1       =  -9;
        static const uint32_t B_DCC1N      = -10;
        static const uint32_t B_DCC0N_T0   = -11;
        static const uint32_t B_DCC1N_T1   = -12;
        static const uint32_t B_T2_T3      = -13;
        static const uint32_t B_T0_T3      = -14;
        static const uint32_t B_T0_T1_T2   = -15;
        static const uint32_t B_T1_T2_T3   = -16;
        static const uint32_t B_DCC0_T1_T2 = -17;
        static const uint32_t B_DCC1_T0_T3 = -18;
        static const uint32_t C_0          = -19;
        static const uint32_t C_1          = -20;

        uint32_t openRow;
        uint8_t  bank;
        uint8_t  bankgr;

        Tick colAllowedAt;
        Tick preAllowedAt;
        Tick actAllowedAt;

        uint32_t rowAccesses;
        uint32_t bytesAccessed;

        Bank()
            : openRow(NO_ROW), bank(0), bankgr(0),
              colAllowedAt(0), preAllowedAt(0), actAllowedAt(0),
              rowAccesses(0), bytesAccessed(0)
        { }
    };

    // ----------------------------------------------------------------
    // Rank class
    // ----------------------------------------------------------------

    /**
     * Rank manages per-rank refresh and power-state machines, DRAMPower
     * accounting, and the bank vector for this rank.
     */
    class Rank : public EventManager
    {
      private:
        enum PowerState {
            PWR_IDLE    = 0,
            PWR_REF,
            PWR_PRE_PDN,
            PWR_ACT,
            PWR_ACT_PDN
        };

        enum RefreshState {
            REF_IDLE = 0,
            REF_DRAIN,
            REF_PRE,
            REF_RUN
        };

        DRAMInterface& dram;

        PowerState pwrStateTrans;
        PowerState pwrState;
        Tick       pwrStateTick;

        RefreshState refreshState;
        Tick         refreshDueAt;

        // Per-rank energy/power stats
        Stats::Scalar actEnergy;
        Stats::Scalar preEnergy;
        Stats::Scalar readEnergy;
        Stats::Scalar writeEnergy;
        Stats::Scalar refreshEnergy;
        Stats::Scalar actBackEnergy;
        Stats::Scalar preBackEnergy;
        Stats::Scalar totalEnergy;
        Stats::Scalar averagePower;
        Stats::Vector pwrStateTime;

        void updatePowerStats();
        void schedulePowerEvent(PowerState pwr_state, Tick tick);

      public:
        uint8_t rank;

        DRAMPower power;

        std::vector<Bank> banks;
        unsigned int numBanksActive;

        std::deque<Tick> actTicks;

        Rank(DRAMInterface& _dram, const DRAMInterfaceParams* _p);

        const std::string name() const;

        void startup(Tick ref_tick);
        void suspend();

        bool isAvailable() const { return refreshState == REF_IDLE; }

        void checkDrainDone();

        void regStats();

        void processActivateEvent();
        EventWrapper<Rank, &Rank::processActivateEvent>  activateEvent;

        void processPrechargeEvent();
        EventWrapper<Rank, &Rank::processPrechargeEvent> prechargeEvent;

        void processRefreshEvent();
        EventWrapper<Rank, &Rank::processRefreshEvent>   refreshEvent;

        void processPowerEvent();
        EventWrapper<Rank, &Rank::processPowerEvent>     powerEvent;
    };

    // ----------------------------------------------------------------
    // DRAMInterface public interface used by MemCtrl / HBMCtrl
    // ----------------------------------------------------------------

    /**
     * Decode a packet address into rank/bank/row and allocate a MemPacket.
     * @param pkt      Original system packet (for packet metadata).
     * @param pkt_addr Address of this DRAM burst within pkt.
     * @param size     Size of this burst in bytes.
     * @param is_read  True for reads, false for writes.
     */
    MemPacket* decodePacket(PacketPtr pkt, Addr pkt_addr,
                            unsigned int size, bool is_read);

    /**
     * Perform the DRAM burst access: update bank/rank timing, schedule
     * activates/precharges, record DRAMPower commands.
     * @param mem_pkt Packet to service (front of read or write queue).
     * @param cmd_at  Earliest tick the column command can issue.
     * @param rdQueue Current read queue (for close/open-adaptive policy).
     * @param wrQueue Current write queue (for close/open-adaptive policy).
     * @return        Tick at which data is ready (readyTime for reads).
     */
    Tick doBurstAccess(MemPacket* mem_pkt, Tick cmd_at,
                       const std::deque<MemPacket*>& rdQueue,
                       const std::deque<MemPacket*>& wrQueue);

    /**
     * FR-FCFS reordering: promote the best candidate to queue.front().
     * @param queue          Read or write queue to reorder.
     * @param extra_col_delay Extra delay (e.g. bus turnaround penalty).
     * @return true if a schedulable packet was found and promoted.
     */
    bool chooseNextFRFCFS(std::deque<MemPacket*>& queue,
                          Tick extra_col_delay) const;

    /** Returns true when all ranks are currently refreshing. */
    bool allRanksBusy() const;

    /** Let each rank know whether it can proceed from REF_DRAIN. */
    void checkDrainState();

    /** Startup: schedule the first refresh event for each rank. */
    void startup(Tick ref_tick);

    /** Suspend: cancel pending refresh events (used during drain). */
    void suspend();

    // ----------------------------------------------------------------
    // MIMDRAM in-DRAM compute primitives
    // (ported from DRAMCtrl; operate on Bank& and Rank& owned by this interface)
    // ----------------------------------------------------------------

    int  pudComputeN(uint32_t row_first, uint32_t row_last,
                     int rowsPerSubarray);
    void pudEnforceActConstraints(Rank& rank_ref, Bank& bank_ref,
                                  Tick act_tick, int bpr, bool bgArch,
                                  Tick tRRD_, Tick tRRD_L_, Tick tXAW_);

    void apBank  (Rank& rank_ref, Bank& bank_ref, Tick act_tick, uint32_t row);
    void aapBank (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                  uint32_t row1, uint32_t row2, bool act_overlapped);
    void aaapBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                  uint32_t row1, uint32_t row2, uint32_t row3);
    void anapBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                  uint32_t row1, uint32_t row2);
    void aaaaapBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                    uint32_t r1, uint32_t r2, uint32_t r3,
                    uint32_t r4, uint32_t r5);

    void rowcloneBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_src, uint32_t row_dst);
    void mrcBank     (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_first, uint32_t row_last);
    void majBank     (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_first, uint32_t row_last);
    void bulkWriteBank(Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                       uint32_t row_first, uint32_t row_last);
    void notXsubBank (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_src, uint32_t row_dst);
    void andXsubBank (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_ref, uint32_t row_com);
    void orXsubBank  (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row_ref, uint32_t row_com);
    void fracBank    (Rank& rank_ref, Bank& bank_ref, Tick act_tick,
                      uint32_t row);

    // ----------------------------------------------------------------
    // Geometry and timing constants (public, read by MemCtrl / HBMCtrl)
    // ----------------------------------------------------------------

    const uint32_t deviceSize;
    const uint32_t deviceBusWidth;
    const uint32_t burstLength;
    const uint32_t deviceRowBufferSize;
    const uint32_t devicesPerRank;
    const uint32_t burstSize;
    const uint32_t rowBufferSize;
    const uint32_t columnsPerRowBuffer;
    uint32_t columnsPerStripe;
    const uint32_t ranksPerChannel;
    const uint32_t bankGroupsPerRank;
    const bool     bankGroupArch;
    const uint32_t banksPerRank;
    const uint32_t channels;
    uint32_t rowsPerBank;
    const uint32_t rowsPerSubarray;

    const Tick M5_CLASS_VAR_USED tCK;
    const Tick tWTR;
    const Tick tRTW;
    const Tick tCS;
    const Tick tBURST;
    const Tick tCCD_L;
    const Tick tRCD;
    const Tick tCL;
    const Tick tRP;
    const Tick tRAS;
    const Tick tWR;
    const Tick tRTP;
    const Tick tRFC;
    const Tick tREFI;
    const Tick tRRD;
    const Tick tRRD_L;
    const Tick tXAW;
    const Tick tWL;
    const Tick tWLOV;
    const Tick tNOT;
    const Tick pudTRAS_viol;
    const Tick pudTRP_viol;
    const int  pudFracIters;
    const uint32_t activationLimit;

    Enums::AddrMap   addrMapping;
    Enums::PageManage pageMgmt;
    const uint32_t   maxAccessesPerRow;

    const bool dll;

    // Ranks owned by this interface
    std::vector<Rank*> ranks;

    // Timestamp offset for DRAMPower (set at startup)
    uint64_t timeStampOffset;

    // Back-reference to the owning MemCtrl (set by MemCtrl::init()).
    // Used by Rank::processPowerEvent to restart the scheduler after refresh.
    MemCtrl* ctrl;

    // ----------------------------------------------------------------
    // DRAMInterface lifecycle
    // ----------------------------------------------------------------

    DRAMInterface(const DRAMInterfaceParams* p);

    void init() M5_ATTR_OVERRIDE;
    void startup() M5_ATTR_OVERRIDE;

    void regStats() M5_ATTR_OVERRIDE;

    /**
     * Helper used by MemCtrl to DRAMPower sort comparison.
     */
    static bool sortTime(const Data::MemCommand& m1,
                         const Data::MemCommand& m2)
    {
        return m1.getTime() < m2.getTime();
    }

  private:

    // ----------------------------------------------------------------
    // Internal helpers
    // ----------------------------------------------------------------

    void activateBank(Rank& rank_ref, Bank& bank_ref,
                      Tick act_tick, uint32_t row);

    void prechargeBank(Rank& rank_ref, Bank& bank_ref,
                       Tick pre_at, bool trace = true);

    /**
     * Find the earliest banks ready to activate for queued requests.
     * Returns a bitmask of candidate banks and a flag indicating whether
     * any of them can issue behind the scenes (hidden bank prep).
     */
    std::pair<uint64_t, bool>
    minBankPrep(const std::deque<MemPacket*>& queue, Tick min_col_at) const;

    // Per-interface stats (bandwidth/latency tracked by MemCtrl; these are
    // device-side energy/row-hit stats)
    Stats::Scalar readRowHits;
    Stats::Scalar writeRowHits;
    Stats::Scalar bytesReadDRAM;
    Stats::Scalar bytesWritten;
    Stats::Scalar bytesPerActivateSamples;
    Stats::Histogram bytesPerActivate;
    Stats::Formula readRowHitRate;
    Stats::Formula writeRowHitRate;
    // References to MemCtrl counters updated during doBurstAccess
    // (stats that span both scheduler and device are owned by MemCtrl
    //  and passed in by pointer where needed)
};

#endif // __MEM_DRAM_INTERFACE_HH__
