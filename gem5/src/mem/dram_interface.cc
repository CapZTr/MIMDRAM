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

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMPower.hh"
#include "debug/DRAMState.hh"
#include "mem/dram_interface.hh"
#include "mem/mem_ctrl.hh"

using namespace std;
using namespace Data;

// ============================================================
// DRAMInterface constructor
// ============================================================

DRAMInterface::DRAMInterface(const DRAMInterfaceParams* p)
    : SimObject(p),
      deviceSize(p->device_size),
      deviceBusWidth(p->device_bus_width),
      burstLength(p->burst_length),
      deviceRowBufferSize(p->device_rowbuffer_size),
      devicesPerRank(p->devices_per_rank),
      burstSize((p->devices_per_rank * p->burst_length * p->device_bus_width) / 8),
      rowBufferSize(p->devices_per_rank * p->device_rowbuffer_size),
      columnsPerRowBuffer(rowBufferSize / burstSize),
      // columnsPerStripe is set in init() when we know the address range
      columnsPerStripe(1),
      ranksPerChannel(p->ranks_per_channel),
      bankGroupsPerRank(p->bank_groups_per_rank),
      bankGroupArch(p->bank_groups_per_rank > 0),
      banksPerRank(p->banks_per_rank),
      channels(p->channels),
      rowsPerBank(0),
      rowsPerSubarray(p->rows_per_subarray),
      tCK(p->tCK),
      tWTR(p->tWTR), tRTW(p->tRTW), tCS(p->tCS),
      tBURST(p->tBURST), tCCD_L(p->tCCD_L),
      tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP), tRAS(p->tRAS),
      tWR(p->tWR), tRTP(p->tRTP),
      tRFC(p->tRFC), tREFI(p->tREFI),
      tRRD(p->tRRD), tRRD_L(p->tRRD_L),
      tXAW(p->tXAW),
      tWL(p->tWL), tWLOV(p->tWLOV), tNOT(p->tNOT),
      pudTRAS_viol(p->pud_tRAS_violated),
      pudTRP_viol(p->pud_tRP_violated),
      pudFracIters(p->pud_frac_iters),
      activationLimit(p->activation_limit),
      addrMapping(p->addr_mapping),
      pageMgmt(p->page_policy),
      maxAccessesPerRow(p->max_accesses_per_row),
      dll(p->dll),
      timeStampOffset(0),
      ctrl(NULL)
{
    fatal_if(!isPowerOf2(ranksPerChannel),
             "DRAM rank count of %d is not allowed, must be a power of two\n",
             ranksPerChannel);
    fatal_if(!isPowerOf2(burstSize),
             "DRAM burst size %d is not allowed, must be a power of two\n",
             burstSize);

    for (int i = 0; i < ranksPerChannel; i++) {
        Rank* rank = new Rank(*this, p);
        ranks.push_back(rank);
        rank->actTicks.resize(activationLimit, 0);
        rank->banks.resize(banksPerRank);
        rank->rank = i;

        for (int b = 0; b < banksPerRank; b++) {
            rank->banks[b].bank = b;
            if (bankGroupArch)
                rank->banks[b].bankgr = b % bankGroupsPerRank;
            else
                rank->banks[b].bankgr = b;
        }
    }

    if (tREFI <= tRP || tREFI <= tRFC)
        fatal("tREFI (%d) must be larger than tRP (%d) and tRFC (%d)\n",
              tREFI, tRP, tRFC);

    if (bankGroupArch) {
        if (bankGroupsPerRank > banksPerRank)
            fatal("banks per rank (%d) must be >= bank groups per rank (%d)\n",
                  banksPerRank, bankGroupsPerRank);
        if ((banksPerRank % bankGroupsPerRank) != 0)
            fatal("Banks per rank (%d) must be evenly divisible by bank groups "
                  "per rank (%d)\n", banksPerRank, bankGroupsPerRank);
        if (tCCD_L <= tBURST)
            fatal("tCCD_L (%d) should be larger than tBURST (%d)\n",
                  tCCD_L, tBURST);
        if (tRRD_L < tRRD)
            fatal("tRRD_L (%d) should be >= tRRD (%d)\n", tRRD_L, tRRD);
    }
}

void
DRAMInterface::init()
{
    // rowsPerBank is computed by MemCtrl after it knows the address range;
    // MemCtrl calls back via setRowsPerBank().  Nothing to do here.
}

void
DRAMInterface::startup()
{
    // Called from MemCtrl::startup(); individual rank startup is triggered
    // by MemCtrl after computing timeStampOffset and busBusyUntil.
}

// ============================================================
// Rank constructor and lifecycle
// ============================================================

DRAMInterface::Rank::Rank(DRAMInterface& _dram, const DRAMInterfaceParams* _p)
    : EventManager(&_dram),
      dram(_dram),
      pwrStateTrans(PWR_IDLE), pwrState(PWR_IDLE), pwrStateTick(0),
      refreshState(REF_IDLE), refreshDueAt(0),
      power(_p, false),
      numBanksActive(0),
      activateEvent(*this),
      prechargeEvent(*this),
      refreshEvent(*this),
      powerEvent(*this)
{ }

const std::string
DRAMInterface::Rank::name() const
{
    return csprintf("%s_%d", dram.name(), rank);
}

void
DRAMInterface::Rank::startup(Tick ref_tick)
{
    assert(ref_tick > curTick());
    pwrStateTick = curTick();
    schedule(refreshEvent, ref_tick);
}

void
DRAMInterface::Rank::suspend()
{
    deschedule(refreshEvent);
}

void
DRAMInterface::Rank::checkDrainDone()
{
    if (refreshState == REF_DRAIN) {
        DPRINTF(DRAM, "Refresh drain done, now precharging\n");
        refreshState = REF_PRE;
        schedule(refreshEvent, curTick());
    }
}

void
DRAMInterface::Rank::processActivateEvent()
{
    if (pwrState != PWR_ACT)
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMInterface::Rank::processPrechargeEvent()
{
    if (numBanksActive == 0)
        schedulePowerEvent(PWR_IDLE, curTick());
}

void
DRAMInterface::Rank::processRefreshEvent()
{
    if (refreshState == REF_IDLE) {
        refreshDueAt = curTick();
        refreshState = REF_DRAIN;
        DPRINTF(DRAM, "Refresh due\n");
    }

    if (refreshState == REF_DRAIN) {
        // Wait for any in-flight access to this rank to finish
        if (dram.ctrl &&
            rank == dram.ctrl->activeRank &&
            dram.ctrl->nextReqEventScheduled()) {
            DPRINTF(DRAM, "Refresh awaiting drain\n");
            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    if (refreshState == REF_PRE) {
        if (pwrState != PWR_IDLE) {
            DPRINTF(DRAM, "Precharging all for refresh\n");
            Tick pre_at = curTick();
            for (auto& b : banks)
                pre_at = std::max(b.preAllowedAt, pre_at);

            Tick act_allowed_at = pre_at + dram.tRP;
            for (auto& b : banks) {
                if (b.openRow != Bank::NO_ROW)
                    dram.prechargeBank(*this, b, pre_at, false);
                else {
                    b.actAllowedAt = std::max(b.actAllowedAt, act_allowed_at);
                    b.preAllowedAt = std::max(b.preAllowedAt, pre_at);
                }
            }
            power.powerlib.doCommand(MemCommand::PREA, 0,
                divCeil(pre_at, dram.tCK) - dram.timeStampOffset);
            DPRINTF(DRAMPower, "%llu,PREA,0,%d\n",
                divCeil(pre_at, dram.tCK) - dram.timeStampOffset, rank);
        } else {
            DPRINTF(DRAM, "All banks precharged, starting refresh\n");
            schedulePowerEvent(PWR_REF, curTick());
        }
        refreshState = REF_RUN;
        assert(numBanksActive == 0);
        return;
    }

    if (refreshState == REF_RUN) {
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick ref_done_at = curTick() + dram.tRFC;
        for (auto& b : banks)
            b.actAllowedAt = ref_done_at;

        power.powerlib.doCommand(MemCommand::REF, 0,
            divCeil(curTick(), dram.tCK) - dram.timeStampOffset);

        sort(power.powerlib.cmdList.begin(),
             power.powerlib.cmdList.end(), DRAMInterface::sortTime);
        power.powerlib.updateCounters(false);
        power.powerlib.calcEnergy();
        updatePowerStats();

        DPRINTF(DRAMPower, "%llu,REF,0,%d\n",
            divCeil(curTick(), dram.tCK) - dram.timeStampOffset, rank);

        if (refreshDueAt + dram.tREFI < ref_done_at)
            fatal("Refresh delayed so long we cannot catch up\n");

        schedule(refreshEvent, refreshDueAt + dram.tREFI - dram.tRP);

        assert(!powerEvent.scheduled());
        schedulePowerEvent(PWR_IDLE, ref_done_at);

        DPRINTF(DRAMState, "Refresh done at %llu, next at %llu\n",
                ref_done_at, refreshDueAt + dram.tREFI);
    }
}

void
DRAMInterface::Rank::schedulePowerEvent(PowerState pwr_state, Tick tick)
{
    assert(tick >= curTick());
    if (!powerEvent.scheduled()) {
        DPRINTF(DRAMState, "Scheduling power event at %llu to state %d\n",
                tick, pwr_state);
        pwrStateTrans = pwr_state;
        schedule(powerEvent, tick);
    } else {
        panic("Scheduled power event at %llu to state %d, "
              "with already scheduled event at %llu to %d\n",
              tick, pwr_state, powerEvent.when(), pwrStateTrans);
    }
}

void
DRAMInterface::Rank::processPowerEvent()
{
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;

    pwrStateTime[prev_state] += duration;
    pwrState     = pwrStateTrans;
    pwrStateTick = curTick();

    if (pwrState == PWR_IDLE) {
        DPRINTF(DRAMState, "All banks precharged\n");
        if (prev_state == PWR_REF) {
            DPRINTF(DRAMState, "Was refreshing for %llu ticks\n", duration);
            assert(pwrState == PWR_IDLE);
            refreshState = REF_IDLE;
            // Kick the scheduler back into action
            if (dram.ctrl)
                dram.ctrl->scheduleNextReq(curTick());
        } else {
            assert(prev_state == PWR_ACT);
            if (refreshState == REF_RUN)
                pwrState = PWR_REF;
        }
    }

    if (pwrState == PWR_REF) {
        DPRINTF(DRAMState, "Refreshing\n");
        assert(refreshState == REF_RUN);
        processRefreshEvent();
    }
}

void
DRAMInterface::Rank::updatePowerStats()
{
    Data::MemoryPowerModel::Energy energy = power.powerlib.getEnergy();
    Data::MemoryPowerModel::Power  rpow   = power.powerlib.getPower();

    actEnergy     = energy.act_energy      * dram.devicesPerRank;
    preEnergy     = energy.pre_energy      * dram.devicesPerRank;
    readEnergy    = energy.read_energy     * dram.devicesPerRank;
    writeEnergy   = energy.write_energy    * dram.devicesPerRank;
    refreshEnergy = energy.ref_energy      * dram.devicesPerRank;
    actBackEnergy = energy.act_stdby_energy * dram.devicesPerRank;
    preBackEnergy = energy.pre_stdby_energy * dram.devicesPerRank;
    totalEnergy   = energy.total_energy    * dram.devicesPerRank;
    averagePower  = rpow.average_power     * dram.devicesPerRank;
}

void
DRAMInterface::Rank::regStats()
{
    using namespace Stats;

    pwrStateTime
        .init(5)
        .name(name() + ".memoryStateTime")
        .desc("Time in different power states");
    pwrStateTime.subname(0, "IDLE");
    pwrStateTime.subname(1, "REF");
    pwrStateTime.subname(2, "PRE_PDN");
    pwrStateTime.subname(3, "ACT");
    pwrStateTime.subname(4, "ACT_PDN");

    actEnergy.name(name() + ".actEnergy")
        .desc("Energy for activate commands per rank (pJ)");
    preEnergy.name(name() + ".preEnergy")
        .desc("Energy for precharge commands per rank (pJ)");
    readEnergy.name(name() + ".readEnergy")
        .desc("Energy for read commands per rank (pJ)");
    writeEnergy.name(name() + ".writeEnergy")
        .desc("Energy for write commands per rank (pJ)");
    refreshEnergy.name(name() + ".refreshEnergy")
        .desc("Energy for refresh commands per rank (pJ)");
    actBackEnergy.name(name() + ".actBackEnergy")
        .desc("Energy for active background per rank (pJ)");
    preBackEnergy.name(name() + ".preBackEnergy")
        .desc("Energy for precharge background per rank (pJ)");
    totalEnergy.name(name() + ".totalEnergy")
        .desc("Total energy per rank (pJ)");
    averagePower.name(name() + ".averagePower")
        .desc("Core power per rank (mW)");
}

void
DRAMInterface::regStats()
{
    SimObject::regStats();
    for (auto r : ranks)
        r->regStats();
}

// ============================================================
// Address decode
// ============================================================

MemPacket*
DRAMInterface::decodePacket(PacketPtr pkt, Addr pkt_addr,
                             unsigned int size, bool is_read)
{
    uint8_t  rank;
    uint8_t  bank;
    uint64_t row;

    Addr addr = pkt_addr / burstSize;

    if (addrMapping == Enums::RoRaBaChCo) {
        addr = addr / columnsPerRowBuffer;
        addr = addr / channels;
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;
        row  = addr % rowsPerBank;
    } else if (addrMapping == Enums::RoRaBaCoCh) {
        addr = addr / columnsPerStripe;
        addr = addr / channels;
        addr = addr / (columnsPerRowBuffer / columnsPerStripe);
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;
        row  = addr % rowsPerBank;
    } else if (addrMapping == Enums::RoCoRaBaCh) {
        addr = addr / columnsPerStripe;
        addr = addr / channels;
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;
        addr = addr / (columnsPerRowBuffer / columnsPerStripe);
        row  = addr % rowsPerBank;
    } else {
        panic("Unknown address mapping policy\n");
    }

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row  < rowsPerBank);
    assert(row  < Bank::NO_ROW);

    DPRINTF(DRAM, "Address: %lld Rank %d Bank %d Row %d\n",
            pkt_addr, rank, bank, row);

    uint16_t bank_id = banksPerRank * rank + bank;
    return new MemPacket(pkt, is_read, rank, bank, (uint32_t)row,
                         bank_id, pkt_addr, size);
}

// ============================================================
// activateBank / prechargeBank
// ============================================================

void
DRAMInterface::activateBank(Rank& rank_ref, Bank& bank_ref,
                             Tick act_tick, uint32_t row)
{
    assert(rank_ref.actTicks.size() == activationLimit);
    DPRINTF(DRAM, "Activate at tick %d\n", act_tick);

    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = row;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;

    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);

    DPRINTF(DRAM, "Activate bank %d, rank %d at tick %lld, now %d active\n",
            bank_ref.bank, rank_ref.rank, act_tick, rank_ref.numBanksActive);

    rank_ref.power.powerlib.doCommand(MemCommand::ACT, bank_ref.bank,
        divCeil(act_tick, tCK) - timeStampOffset);
    DPRINTF(DRAMPower, "%llu,ACT,%d,%d\n",
            divCeil(act_tick, tCK) - timeStampOffset,
            bank_ref.bank, rank_ref.rank);

    bank_ref.preAllowedAt = act_tick + tRAS;
    bank_ref.colAllowedAt = std::max(act_tick + tRCD, bank_ref.colAllowedAt);

    for (int i = 0; i < banksPerRank; i++) {
        if (bankGroupArch && (bank_ref.bankgr == rank_ref.banks[i].bankgr))
            rank_ref.banks[i].actAllowedAt =
                std::max(act_tick + tRRD_L, rank_ref.banks[i].actAllowedAt);
        else
            rank_ref.banks[i].actAllowedAt =
                std::max(act_tick + tRRD, rank_ref.banks[i].actAllowedAt);
    }

    if (!rank_ref.actTicks.empty()) {
        if (rank_ref.actTicks.back() &&
            (act_tick - rank_ref.actTicks.back()) < tXAW)
            panic("Got %d activates in window %d which is smaller than %llu\n",
                  activationLimit,
                  act_tick - rank_ref.actTicks.back(), tXAW);

        rank_ref.actTicks.pop_back();
        rank_ref.actTicks.push_front(act_tick);

        if (rank_ref.actTicks.back() &&
            (act_tick - rank_ref.actTicks.back()) < tXAW) {
            DPRINTF(DRAM, "Enforcing tXAW with X=%d\n", activationLimit);
            for (int j = 0; j < banksPerRank; j++)
                rank_ref.banks[j].actAllowedAt =
                    std::max(rank_ref.actTicks.back() + tXAW,
                             rank_ref.banks[j].actAllowedAt);
        }
    }

    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
}

void
DRAMInterface::prechargeBank(Rank& rank_ref, Bank& bank_ref,
                              Tick pre_at, bool trace)
{
    assert(bank_ref.openRow != Bank::NO_ROW);
    bytesPerActivate.sample(bank_ref.bytesAccessed);

    bank_ref.openRow      = Bank::NO_ROW;
    bank_ref.preAllowedAt = pre_at;

    Tick pre_done_at = pre_at + tRP;
    bank_ref.actAllowedAt = std::max(bank_ref.actAllowedAt, pre_done_at);

    assert(rank_ref.numBanksActive != 0);
    --rank_ref.numBanksActive;

    DPRINTF(DRAM, "Precharging bank %d, rank %d at tick %lld, now %d active\n",
            bank_ref.bank, rank_ref.rank, pre_at, rank_ref.numBanksActive);

    if (trace) {
        rank_ref.power.powerlib.doCommand(MemCommand::PRE, bank_ref.bank,
            divCeil(pre_at, tCK) - timeStampOffset);
        DPRINTF(DRAMPower, "%llu,PRE,%d,%d\n",
                divCeil(pre_at, tCK) - timeStampOffset,
                bank_ref.bank, rank_ref.rank);
    }

    if (rank_ref.numBanksActive == 0) {
        if (!rank_ref.prechargeEvent.scheduled())
            schedule(rank_ref.prechargeEvent, pre_done_at);
        else if (rank_ref.prechargeEvent.when() > pre_done_at)
            reschedule(rank_ref.prechargeEvent, pre_done_at);
    }
}

// ============================================================
// MIMDRAM primitives
// ============================================================

int
DRAMInterface::pudComputeN(uint32_t row_first, uint32_t row_last,
                            int rowsPerSub)
{
    int sub_first = row_first / rowsPerSub;
    int sub_last  = row_last  / rowsPerSub;
    int sub_lo    = std::min(sub_first, sub_last);
    int sub_hi    = std::max(sub_first, sub_last);
    // N = number of rows from the boundary between the two subarrays
    // that are simultaneously activated by the APA predecoder.
    int pos_lo = row_first % rowsPerSub;
    int pos_hi = rowsPerSub - 1 - (row_last % rowsPerSub);
    (void)sub_lo; (void)sub_hi;
    return std::min(pos_lo, pos_hi) + 1;
}

void
DRAMInterface::pudEnforceActConstraints(Rank& rank_ref, Bank& bank_ref,
                                         Tick act_tick, int bpr, bool bgArch,
                                         Tick tRRD_, Tick tRRD_L_, Tick tXAW_)
{
    for (int i = 0; i < bpr; i++) {
        if (bgArch && bank_ref.bankgr == rank_ref.banks[i].bankgr)
            rank_ref.banks[i].actAllowedAt =
                std::max(act_tick + tRRD_L_, rank_ref.banks[i].actAllowedAt);
        else
            rank_ref.banks[i].actAllowedAt =
                std::max(act_tick + tRRD_, rank_ref.banks[i].actAllowedAt);
    }
    // tXAW window enforcement (simplified: advance all banks)
    if (tXAW_ > 0) {
        for (int j = 0; j < bpr; j++)
            rank_ref.banks[j].actAllowedAt =
                std::max(act_tick + tXAW_, rank_ref.banks[j].actAllowedAt);
    }
}

void
DRAMInterface::apBank(Rank& rank_ref, Bank& bank_ref,
                       Tick act_tick, uint32_t row)
{
    // Single ACT + PRE (used as building block)
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = row;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::aapBank(Rank& rank_ref, Bank& bank_ref,
                        Tick act_tick, uint32_t row1, uint32_t row2,
                        bool act_overlapped)
{
    // Activate-Activate-Precharge (AAP): SIMDRAM / MIMDRAM building block.
    // act_overlapped=true uses tWLOV (shorter) between the two ACTs.
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);

    Tick act2_at = act_tick + (act_overlapped ? tWLOV : tWL);
    bank_ref.preAllowedAt = act2_at + tRAS;

    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);

    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);

    // After AAP, bank.actAllowedAt reflects end of precharge
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::aaapBank(Rank& rank_ref, Bank& bank_ref,
                         Tick act_tick, uint32_t row1, uint32_t row2,
                         uint32_t row3)
{
    // ACT+ACT+ACT+PRE: 5ns between each ACT, PRE after tRAS + 2*tWLOV
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS + 2 * tWLOV;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::anapBank(Rank& rank_ref, Bank& bank_ref,
                         Tick act_tick, uint32_t row1, uint32_t row2)
{
    // ACT + NOT(tNOT after tRCD) + ACT + PRE
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRCD + tNOT + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::aaaaapBank(Rank& rank_ref, Bank& bank_ref,
                           Tick act_tick,
                           uint32_t r1, uint32_t r2, uint32_t r3,
                           uint32_t r4, uint32_t r5)
{
    // ACTx5 + PRE: 5ns between each ACT, PRE after tRAS + 4*tWLOV
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS + 4 * tWLOV;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::rowcloneBank(Rank& rank_ref, Bank& bank_ref,
                             Tick act_tick,
                             uint32_t row_src, uint32_t row_dst)
{
    // RowClone (COTS): ACT src → PRE(viol.tRP) → ACT dst → tRAS → PRE
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::mrcBank(Rank& rank_ref, Bank& bank_ref,
                        Tick act_tick,
                        uint32_t row_first, uint32_t row_last)
{
    // Multi-Row Copy: ACT rf → tRAS → PRE(viol.tRP) → ACT rl → tRAS → PRE
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::majBank(Rank& rank_ref, Bank& bank_ref,
                        Tick act_tick,
                        uint32_t row_first, uint32_t row_last)
{
    // Majority/charge-sharing: ACT rf(viol.tRAS) → PRE(viol.tRP) → ACT rl → tRAS → PRE
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + pudTRAS_viol + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::bulkWriteBank(Rank& rank_ref, Bank& bank_ref,
                              Tick act_tick,
                              uint32_t row_first, uint32_t row_last)
{
    // BULK_WRITE = MAJ sequence followed by a WRITE of write_data to all N rows
    majBank(rank_ref, bank_ref, act_tick, row_first, row_last);
}

void
DRAMInterface::notXsubBank(Rank& rank_ref, Bank& bank_ref,
                            Tick act_tick,
                            uint32_t row_src, uint32_t row_dst)
{
    // Cross-subarray NOT: ACT src → tRAS → PRE(viol.tRP) → ACT dst → tRAS → PRE
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + tRAS + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::andXsubBank(Rank& rank_ref, Bank& bank_ref,
                            Tick act_tick,
                            uint32_t row_ref, uint32_t row_com)
{
    // Cross-subarray AND: APA(ref, com) → result in com rows
    // FCDRAM §6.2: the AND/OR APA violates BOTH tRAS and tRP; only the
    // final restore waits the full tRAS (NOT keeps a full first tRAS).
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + pudTRAS_viol + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::orXsubBank(Rank& rank_ref, Bank& bank_ref,
                           Tick act_tick,
                           uint32_t row_ref, uint32_t row_com)
{
    // Cross-subarray OR: APA(ref, com) → result in com rows
    // Same double-violated APA timing as andXsubBank (FCDRAM §6.2).
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + pudTRAS_viol + pudTRP_viol + tRAS;
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

void
DRAMInterface::fracBank(Rank& rank_ref, Bank& bank_ref,
                         Tick act_tick, uint32_t row)
{
    // Frac (VDD/2 init): (ACT → PRE_viol) x pudFracIters
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow       = Bank::DOUBLE_ROW;
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses   = 0;
    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);
    bank_ref.preAllowedAt = act_tick + (Tick)pudFracIters * (pudTRAS_viol + tRP);
    pudEnforceActConstraints(rank_ref, bank_ref, act_tick,
                             banksPerRank, bankGroupArch, tRRD, tRRD_L, tXAW);
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_tick);
    else if (rank_ref.activateEvent.when() > act_tick)
        reschedule(rank_ref.activateEvent, act_tick);
    prechargeBank(rank_ref, bank_ref, bank_ref.preAllowedAt);
}

// ============================================================
// doBurstAccess
// ============================================================

Tick
DRAMInterface::doBurstAccess(MemPacket* mem_pkt, Tick cmd_at,
                              const std::deque<MemPacket*>& rdQueue,
                              const std::deque<MemPacket*>& wrQueue)
{
    DPRINTF(DRAM, "Timing access to addr %lld, rank/bank/row %d %d %d\n",
            mem_pkt->addr, mem_pkt->rank, mem_pkt->bank, mem_pkt->row);

    Rank& rank = *ranks[mem_pkt->rank];
    Bank& bank = rank.banks[mem_pkt->bank];

    bool row_hit = true;

    // ---- row-operation dispatch ----
    if (mem_pkt->is_row_op) {
        if (bank.openRow != Bank::NO_ROW)
            prechargeBank(rank, bank, std::max(bank.preAllowedAt, curTick()));
        cmd_at = std::max(cmd_at, bank.actAllowedAt);

        static const char* rowOpNames[] = {
            "ROWAND","ROWOR","ROWNOT","ROWXOR",
            "ROWAP","ROWAAP","ROWCOPY",
            "ROWANAP","ROWAAAP","ROWAAAAAP",
            "ROWCLONE","MRC","MAJ","BULK_WRITE",
            "NOT_XSUB","AND_XSUB","OR_XSUB","FRAC","MAJ3",
            "ROW_RD_STREAM","ROW_WR_STREAM"
        };
        DPRINTF(DRAM, "RowOp %s rank %d bank %d dst %d src1 %d src2 %d\n",
                rowOpNames[mem_pkt->row_op],
                mem_pkt->rank, mem_pkt->bank,
                mem_pkt->row, mem_pkt->src1_row, mem_pkt->src2_row);

        switch (mem_pkt->row_op) {
            case Request::ROWAND:
                aapBank(rank, bank, cmd_at, mem_pkt->src1_row, Bank::B_T0,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, mem_pkt->src2_row, Bank::B_T1,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::C_0,          Bank::B_T2,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::B_T0_T1_T2,   mem_pkt->row,  true); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWOR:
                aapBank(rank, bank, cmd_at, mem_pkt->src1_row, Bank::B_T0,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, mem_pkt->src2_row, Bank::B_T1,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::C_1,          Bank::B_T2,    true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::B_T0_T1_T2,   mem_pkt->row,  true); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWNOT:
                aapBank(rank, bank, cmd_at, mem_pkt->src1_row, Bank::B_DCC0N, true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::B_DCC0,       mem_pkt->row,  true); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWXOR:
                aapBank(rank, bank, cmd_at, mem_pkt->src1_row, Bank::B_DCC0N_T0, true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, mem_pkt->src2_row, Bank::B_DCC1N_T1, true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::C_0,          Bank::B_T2_T3,    true); cmd_at = bank.actAllowedAt;
                apBank (rank, bank, cmd_at, Bank::B_DCC0_T1_T2                        ); cmd_at = bank.actAllowedAt;
                apBank (rank, bank, cmd_at, Bank::B_DCC1_T0_T3                        ); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::C_1,          Bank::B_T2,       true); cmd_at = bank.actAllowedAt;
                aapBank(rank, bank, cmd_at, Bank::B_T0_T1_T2,   mem_pkt->row,     true); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWAP:
                apBank(rank, bank, cmd_at, Bank::B_T0_T1_T2); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWAAP:
                aapBank(rank, bank, cmd_at, 0, 0, true); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWANAP:
                anapBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWAAAP:
                aaapBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->src2_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWAAAAAP:
                aaaaapBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->src2_row, mem_pkt->row, mem_pkt->row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::ROWCLONE:
                rowcloneBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::MRC:
                mrcBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::MAJ:
                majBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::BULK_WRITE:
                bulkWriteBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::NOT_XSUB:
                notXsubBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::AND_XSUB:
                andXsubBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::OR_XSUB:
                orXsubBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::FRAC:
                fracBank(rank, bank, cmd_at, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            case Request::MAJ3:
                // In-place 3-input MAJ: same charge-sharing MAJ timing as MAJ.
                majBank(rank, bank, cmd_at, mem_pkt->src1_row, mem_pkt->row); cmd_at = bank.actAllowedAt;
                break;
            // Cross-channel row-copy halves (see dram_ctrl.cc for the full
            // rationale): one channel-side full-row stream each — ACT,
            // tRCD, columnsPerRowBuffer bursts, tRTP/tWR recovery, PRE.
            // This interface path has no per-channel stream-bus guard
            // (rowStreamBusUntil lives in DRAMCtrl, the measured player
            // path); per-bank chaining still applies.
            case Request::ROW_RD_STREAM: {
                activateBank(rank, bank, cmd_at, mem_pkt->row);
                Tick col_at = bank.colAllowedAt;
                Tick end_at = col_at + columnsPerRowBuffer * tBURST;
                Tick pre_at = std::max(bank.preAllowedAt,
                                       col_at + (columnsPerRowBuffer - 1) * tBURST + tRTP);
                prechargeBank(rank, bank, pre_at);
                cmd_at = end_at;
                break;
            }
            case Request::ROW_WR_STREAM: {
                activateBank(rank, bank, cmd_at, mem_pkt->row);
                Tick col_at = bank.colAllowedAt;
                Tick end_at = col_at + columnsPerRowBuffer * tBURST;
                Tick pre_at = std::max(bank.preAllowedAt,
                                       col_at + (columnsPerRowBuffer - 1) * tBURST + tWR);
                prechargeBank(rank, bank, pre_at);
                cmd_at = end_at;
                break;
            }
            case Request::ROWCOPY: {
                // Inter-bank / inter-rank row copy: read src, write dst
                Rank& src_rank_ref = *ranks[mem_pkt->src_rank];
                Bank& src_bank_ref = src_rank_ref.banks[mem_pkt->src_bank];

                if (src_bank_ref.openRow != Bank::NO_ROW)
                    prechargeBank(src_rank_ref, src_bank_ref,
                                  std::max(src_bank_ref.preAllowedAt, curTick()));

                Tick src_act_at = std::max(cmd_at, src_bank_ref.actAllowedAt);
                if (src_rank_ref.prechargeEvent.scheduled())
                    src_act_at = std::max(src_act_at, src_rank_ref.prechargeEvent.when() + 1);
                if (src_rank_ref.powerEvent.scheduled())
                    src_act_at = std::max(src_act_at, src_rank_ref.powerEvent.when() + 1);
                activateBank(src_rank_ref, src_bank_ref, src_act_at, mem_pkt->src1_row);

                Tick dst_act_at = std::max(src_act_at, bank.actAllowedAt);
                if (&rank != &src_rank_ref) {
                    if (rank.prechargeEvent.scheduled())
                        dst_act_at = std::max(dst_act_at, rank.prechargeEvent.when() + 1);
                    if (rank.powerEvent.scheduled())
                        dst_act_at = std::max(dst_act_at, rank.powerEvent.when() + 1);
                }
                activateBank(rank, bank, dst_act_at, mem_pkt->row);

                Tick rd_col_at  = src_bank_ref.colAllowedAt;
                Tick rd_end_at  = rd_col_at + columnsPerRowBuffer * tBURST;
                Tick src_pre_at = std::max(src_bank_ref.preAllowedAt,
                                           rd_col_at + (columnsPerRowBuffer - 1) * tBURST + tRTP);
                prechargeBank(src_rank_ref, src_bank_ref, src_pre_at);

                Tick wr_col_at  = std::max(bank.colAllowedAt, rd_end_at + tRTW);
                Tick wr_end_at  = wr_col_at + columnsPerRowBuffer * tBURST;
                Tick dst_pre_at = std::max(bank.preAllowedAt,
                                           wr_col_at + (columnsPerRowBuffer - 1) * tBURST + tWR);
                prechargeBank(rank, bank, dst_pre_at);

                cmd_at = wr_end_at;

                DPRINTF(DRAM, "ROWCOPY rank%d/bank%d/row%d -> rank%d/bank%d/row%d "
                              "rd@%llu wr@%llu\n",
                        mem_pkt->src_rank, mem_pkt->src_bank, mem_pkt->src1_row,
                        mem_pkt->rank, mem_pkt->bank, mem_pkt->row,
                        rd_col_at, wr_col_at);
                break;
            }
            default:
                panic("Unknown row op %d\n", mem_pkt->row_op);
        }

        mem_pkt->readyTime = cmd_at + tCL;
        return mem_pkt->readyTime;
    }

    // ---- normal read/write ----
    if (bank.openRow == mem_pkt->row) {
        // row hit - nothing to do
    } else {
        row_hit = false;
        if (bank.openRow != Bank::NO_ROW)
            prechargeBank(rank, bank, std::max(bank.preAllowedAt, curTick()));
        Tick act_tick = std::max(bank.actAllowedAt, curTick());
        activateBank(rank, bank, act_tick, mem_pkt->row);
        cmd_at = bank.colAllowedAt;
    }

    // Respect bus-busy constraint (passed in as cmd_at from MemCtrl)
    // cmd_at here already incorporates busBusyUntil from the caller.
    mem_pkt->readyTime = cmd_at + tCL + tBURST;

    // Update CAS-to-CAS delay for all banks
    Tick cmd_dly;
    for (int j = 0; j < ranksPerChannel; j++) {
        for (int i = 0; i < banksPerRank; i++) {
            if (mem_pkt->rank == (uint8_t)j) {
                if (bankGroupArch &&
                    bank.bankgr == ranks[j]->banks[i].bankgr)
                    cmd_dly = tCCD_L;
                else
                    cmd_dly = tBURST;
            } else {
                cmd_dly = tBURST + tCS;
            }
            ranks[j]->banks[i].colAllowedAt =
                std::max(cmd_at + cmd_dly, ranks[j]->banks[i].colAllowedAt);
        }
    }

    bank.preAllowedAt = std::max(bank.preAllowedAt,
                                 mem_pkt->isRead
                                 ? cmd_at + tRTP
                                 : mem_pkt->readyTime + tWR);

    bank.bytesAccessed += burstSize;
    ++bank.rowAccesses;

    bool auto_precharge = (pageMgmt == Enums::close) ||
                          (bank.rowAccesses == maxAccessesPerRow);

    if (!auto_precharge &&
        (pageMgmt == Enums::open_adaptive ||
         pageMgmt == Enums::close_adaptive)) {
        bool got_more_hits    = false;
        bool got_bank_conflict = false;
        const deque<MemPacket*>& queue = mem_pkt->isRead ? rdQueue : wrQueue;
        auto p = queue.begin();
        ++p; // skip current head
        while (!got_more_hits && p != queue.end()) {
            bool same_rb = (mem_pkt->rank == (*p)->rank) &&
                           (mem_pkt->bank == (*p)->bank);
            bool same_row = mem_pkt->row == (*p)->row;
            got_more_hits    |= same_rb && same_row;
            got_bank_conflict |= same_rb && !same_row;
            ++p;
        }
        auto_precharge = !got_more_hits &&
            (got_bank_conflict || pageMgmt == Enums::close_adaptive);
    }

    std::string mem_cmd = mem_pkt->isRead ? "RD" : "WR";
    MemCommand::cmds command = mem_pkt->isRead ? MemCommand::RD : MemCommand::WR;

    if (auto_precharge)
        prechargeBank(rank, bank, std::max(curTick(), bank.preAllowedAt));

    rank.power.powerlib.doCommand(command, mem_pkt->bank,
        divCeil(cmd_at, tCK) - timeStampOffset);
    DPRINTF(DRAMPower, "%llu,%s,%d,%d\n",
            divCeil(cmd_at, tCK) - timeStampOffset,
            mem_cmd, mem_pkt->bank, mem_pkt->rank);

    // Update device-side stats
    if (mem_pkt->isRead) {
        if (row_hit) readRowHits++;
        bytesReadDRAM += burstSize;
    } else {
        if (row_hit) writeRowHits++;
        bytesWritten += burstSize;
    }

    return mem_pkt->readyTime;
}

// ============================================================
// FR-FCFS scheduler helper
// ============================================================

bool
DRAMInterface::chooseNextFRFCFS(std::deque<MemPacket*>& queue,
                                 Tick extra_col_delay) const
{
    // Find the minimum column-access time across all candidates
    Tick min_col_at = MaxTick;
    for (const auto& p : queue) {
        if (ranks[p->rank]->isAvailable())
            min_col_at = std::min(min_col_at,
                                  ranks[p->rank]->banks[p->bank].colAllowedAt);
    }
    min_col_at += extra_col_delay;

    auto selected_pkt_it = queue.end();
    bool found_prepped_pkt   = false;
    bool found_earliest_pkt  = false;
    bool found_hidden_bank   = false;
    bool found_seamless_bank = false;

    // Compute earliest-bank mask for hidden bank prep
    std::pair<uint64_t, bool> bankStatus = minBankPrep(queue, min_col_at);
    uint64_t earliest_banks    = bankStatus.first;
    bool     hidden_bank_prep  = bankStatus.second;

    for (auto i = queue.begin(); i != queue.end(); ++i) {
        const MemPacket* pkt = *i;
        if (!ranks[pkt->rank]->isAvailable())
            continue;

        const Bank& b = ranks[pkt->rank]->banks[pkt->bank];

        bool prepped = (b.openRow == pkt->row);
        bool seamless = (b.colAllowedAt <= min_col_at);

        if (prepped && seamless) {
            selected_pkt_it = i;
            break; // best possible: row hit that fits immediately
        }

        if (prepped && !found_prepped_pkt) {
            found_prepped_pkt = true;
            if (!found_seamless_bank)
                selected_pkt_it = i;
        }

        if (!found_prepped_pkt || found_hidden_bank) {
            if (bits(earliest_banks, pkt->bankId, pkt->bankId)) {
                if (!found_earliest_pkt) {
                    found_earliest_pkt = true;
                    found_hidden_bank  = hidden_bank_prep;
                    if (hidden_bank_prep || !found_prepped_pkt)
                        selected_pkt_it = i;
                }
            }
        }
    }

    if (selected_pkt_it != queue.end()) {
        MemPacket* selected_pkt = *selected_pkt_it;
        queue.erase(selected_pkt_it);
        queue.push_front(selected_pkt);
        return true;
    }
    return false;
}

std::pair<uint64_t, bool>
DRAMInterface::minBankPrep(const std::deque<MemPacket*>& queue,
                            Tick min_col_at) const
{
    uint64_t bank_mask   = 0;
    Tick     min_act_at  = MaxTick;

    const Tick hidden_act_max = std::max(min_col_at - tRCD, curTick());

    bool found_seamless_bank = false;
    bool hidden_bank_prep    = false;

    std::vector<bool> got_waiting(ranksPerChannel * banksPerRank, false);
    for (const auto& p : queue) {
        if (ranks[p->rank]->isAvailable())
            got_waiting[p->bankId] = true;
    }

    for (int i = 0; i < ranksPerChannel; i++) {
        for (int j = 0; j < banksPerRank; j++) {
            uint16_t bank_id = i * banksPerRank + j;
            if (!got_waiting[bank_id]) continue;
            assert(ranks[i]->isAvailable());

            Tick act_at = ranks[i]->banks[j].openRow == Bank::NO_ROW
                ? std::max(ranks[i]->banks[j].actAllowedAt, curTick())
                : std::max(ranks[i]->banks[j].preAllowedAt, curTick()) + tRP;

            Tick col_at = std::max(ranks[i]->banks[j].colAllowedAt,
                                   act_at + tRCD);

            bool new_seamless = col_at <= min_col_at;

            if (new_seamless || (!found_seamless_bank && act_at <= min_act_at)) {
                if (!found_seamless_bank &&
                    (new_seamless || act_at < min_act_at))
                    bank_mask = 0;

                found_seamless_bank |= new_seamless;
                hidden_bank_prep     = act_at <= hidden_act_max;
                replaceBits(bank_mask, bank_id, bank_id, 1);
                min_act_at = act_at;
            }
        }
    }
    return std::make_pair(bank_mask, hidden_bank_prep);
}

// ============================================================
// allRanksBusy / checkDrainState / startup / suspend
// ============================================================

bool
DRAMInterface::allRanksBusy() const
{
    for (auto r : ranks)
        if (r->isAvailable())
            return false;
    return true;
}

void
DRAMInterface::checkDrainState()
{
    for (auto r : ranks)
        r->checkDrainDone();
}

void
DRAMInterface::startup(Tick ref_tick)
{
    for (auto r : ranks)
        r->startup(ref_tick);
}

void
DRAMInterface::suspend()
{
    for (auto r : ranks)
        r->suspend();
}

DRAMInterface*
DRAMInterfaceParams::create()
{
    return new DRAMInterface(this);
}
