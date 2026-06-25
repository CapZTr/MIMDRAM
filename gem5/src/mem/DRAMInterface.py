# Copyright (c) 2012-2015 ARM Limited
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.proxy  import *
from m5.SimObject import SimObject
from MemObject  import *

class DRAMInterface(SimObject):
    type        = 'DRAMInterface'
    cxx_header  = 'mem/dram_interface.hh'

    # ----------------------------------------------------------------
    # Device geometry
    # ----------------------------------------------------------------
    device_size           = Param.MemorySize('Size of a DRAM device/die')
    device_bus_width      = Param.Unsigned('Bus width per device (bits)')
    burst_length          = Param.Unsigned('Burst length (transfers)')
    device_rowbuffer_size = Param.MemorySize('Row-buffer size per device')
    devices_per_rank      = Param.Unsigned('Devices per rank')
    ranks_per_channel     = Param.Unsigned('Ranks per channel')
    bank_groups_per_rank  = Param.Unsigned(0, 'Bank groups per rank (0 = no bank-group arch)')
    banks_per_rank        = Param.Unsigned('Banks per rank')
    channels              = Param.Unsigned(1, 'Number of channels')
    rows_per_subarray     = Param.Unsigned(512, 'Rows per subarray (MIMDRAM PUD)')

    # ----------------------------------------------------------------
    # Timing parameters
    # ----------------------------------------------------------------
    tCK    = Param.Latency('Clock period')
    tWTR   = Param.Latency('Write-to-read turnaround (same rank)')
    tRTW   = Param.Latency('Read-to-write turnaround (same rank)')
    tCS    = Param.Latency('Rank-to-rank switching delay')
    tBURST = Param.Latency('Burst duration')
    tCCD_L = Param.Latency('0ns', 'CAS-to-CAS delay long (same bank group)')
    tRCD   = Param.Latency('RAS-to-CAS delay')
    tCL    = Param.Latency('CAS latency')
    tRP    = Param.Latency('Row precharge time')
    tRAS   = Param.Latency('Row active time')
    tWR    = Param.Latency('Write recovery time')
    tRTP   = Param.Latency('Read-to-precharge time')
    tRFC   = Param.Latency('Refresh cycle time')
    tREFI  = Param.Latency('Refresh interval')
    tRRD   = Param.Latency('ACT-to-ACT delay (different bank groups)')
    tRRD_L = Param.Latency('0ns', 'ACT-to-ACT delay long (same bank group)')
    tXAW   = Param.Latency('X-activation window')
    activation_limit = Param.Unsigned('Max activations per tXAW window')

    # MIMDRAM / PUD timings
    tWL         = Param.Latency('5ns',  'Inter-ACT delay for AAP (SIMDRAM)')
    tWLOV       = Param.Latency('5ns',  'Overlapped inter-ACT delay for AAP')
    tNOT        = Param.Latency('15ns', 'NOT operation duration')
    pud_tRAS_violated = Param.Latency('1ns', 'Violated tRAS for PUD ops')
    pud_tRP_violated  = Param.Latency('1ns', 'Violated tRP for PUD ops')
    pud_frac_iters    = Param.Int(10, 'Frac iterations for VDD/2 init')

    # ----------------------------------------------------------------
    # Page / scheduling policy
    # ----------------------------------------------------------------
    addr_mapping = Param.AddrMap('RoRaBaChCo', 'Address mapping policy')
    page_policy  = Param.PageManage('open_adaptive', 'Page management policy')
    max_accesses_per_row = Param.Unsigned(16,
        'Max column accesses before forced precharge')

    # ----------------------------------------------------------------
    # Misc
    # ----------------------------------------------------------------
    dll = Param.Bool(True, 'DLL enabled')

    # Power model (IDD) parameters - supplied by device configs below.
    # Each DRAMPower param maps 1-to-1 to a DRAMPower IDD entry.
    VDD   = Param.Float(0, 'Main supply voltage (V)')
    VDD2  = Param.Float(0, 'Secondary supply voltage (V) - ignored if 0')
    IDD0  = Param.Float(0, 'Active precharge current (mA)')
    IDD02 = Param.Float(0, 'Active precharge current VDD2 (mA)')
    IDD2P0= Param.Float(0, 'Precharge power-down slow current (mA)')
    IDD2P02=Param.Float(0, 'Precharge power-down slow current VDD2 (mA)')
    IDD2P1= Param.Float(0, 'Precharge power-down fast current (mA)')
    IDD2P12=Param.Float(0, 'Precharge power-down fast current VDD2 (mA)')
    IDD2N = Param.Float(0, 'Precharge standby current (mA)')
    IDD2N2= Param.Float(0, 'Precharge standby current VDD2 (mA)')
    IDD3P0= Param.Float(0, 'Active power-down slow current (mA)')
    IDD3P02=Param.Float(0, 'Active power-down slow current VDD2 (mA)')
    IDD3P1= Param.Float(0, 'Active power-down fast current (mA)')
    IDD3P12=Param.Float(0, 'Active power-down fast current VDD2 (mA)')
    IDD3N = Param.Float(0, 'Active standby current (mA)')
    IDD3N2= Param.Float(0, 'Active standby current VDD2 (mA)')
    IDD4R = Param.Float(0, 'Burst read current (mA)')
    IDD4R2= Param.Float(0, 'Burst read current VDD2 (mA)')
    IDD4W = Param.Float(0, 'Burst write current (mA)')
    IDD4W2= Param.Float(0, 'Burst write current VDD2 (mA)')
    IDD5  = Param.Float(0, 'Refresh current (mA)')
    IDD52 = Param.Float(0, 'Refresh current VDD2 (mA)')
    IDD6  = Param.Float(0, 'Self-refresh current (mA)')
    IDD62 = Param.Float(0, 'Self-refresh current VDD2 (mA)')


# ==================================================================
# HBM 1.0 - single pseudo-channel (128-bit bus, 1 Gb per PC)
# ==================================================================
# HBM_1000_4H_1x128 / HBM_1000_4H_1x64 correspond to the two pseudo-
# channel widths of a single HBM 4-Hi stack (one stack = 8 channels of
# 128b = 2 pseudo-channels x 64b each).
#
# Timings are derived from:
#   Jedec HBM 1.0 spec (JESD235) + Samsung K4G80325FB-A2C datasheet.
# ==================================================================

class HBM_1000_4H_1x128(DRAMInterface):
    """
    HBM 1.0 pseudo-channel: 1 Gb, 128-bit bus, 1000 MHz effective.
    Use this for a single-PC (non-partitioned) HBM model.
    """
    # 128b bus = 16 bytes/transfer, BL=4 -> 64 B burst
    device_size           = '256MB'
    device_bus_width      = 128
    burst_length          = 4
    device_rowbuffer_size = '2kB'
    devices_per_rank      = 1
    ranks_per_channel     = 1
    banks_per_rank        = 16
    # HBM has no bank groups
    bank_groups_per_rank  = 0
    channels              = 1

    tCK    = '1ns'
    tWTR   = '7.5ns'
    tRTW   = '0ns'
    tCS    = '1.25ns'
    tBURST = '4ns'      # 4 transfers x 1 ns
    tCCD_L = '0ns'
    tRCD   = '14ns'
    tCL    = '14ns'
    tRP    = '14ns'
    tRAS   = '33ns'
    tWR    = '14ns'
    tRTP   = '7.5ns'
    tRFC   = '170ns'
    tREFI  = '3900ns'
    tRRD   = '4ns'
    tRRD_L = '0ns'
    tXAW   = '16ns'
    activation_limit = 4
    addr_mapping = 'RoRaBaChCo'
    page_policy  = 'close'

    # IDD values from Samsung K4G80325FB-A2C (8 Gb HBM, per-die)
    VDD    = 1.2
    IDD0   = 60
    IDD02  = 0
    IDD2N  = 26
    IDD2N2 = 0
    IDD3N  = 34
    IDD3N2 = 0
    IDD4W  = 123
    IDD4W2 = 0
    IDD4R  = 123
    IDD4R2 = 0
    IDD5   = 215
    IDD52  = 0
    IDD6   = 12
    IDD62  = 0
    IDD2P1 = 12
    IDD2P12= 0
    IDD3P1 = 20
    IDD3P12= 0
    IDD2P0 = 6
    IDD2P02= 0
    IDD3P0 = 8
    IDD3P02= 0


class HBM_1000_4H_1x64(DRAMInterface):
    """
    HBM 1.0 pseudo-channel: 1 Gb, 64-bit bus (half the bus of HBM_1000_4H_1x128).
    Used when the HBMCtrl splits one physical 128-bit channel into two 64-bit
    pseudo-channels (PC0 and PC1), each managed by a separate DRAMInterface.
    """
    device_size           = '256MB'
    device_bus_width      = 64
    burst_length          = 4
    device_rowbuffer_size = '2kB'
    devices_per_rank      = 1
    ranks_per_channel     = 1
    banks_per_rank        = 8    # 8 banks per pseudo-channel
    bank_groups_per_rank  = 0
    channels              = 1

    tCK    = '1ns'
    tWTR   = '7.5ns'
    tRTW   = '0ns'
    tCS    = '1.25ns'
    tBURST = '4ns'
    tCCD_L = '0ns'
    tRCD   = '14ns'
    tCL    = '14ns'
    tRP    = '14ns'
    tRAS   = '33ns'
    tWR    = '14ns'
    tRTP   = '7.5ns'
    tRFC   = '170ns'
    tREFI  = '3900ns'
    tRRD   = '4ns'
    tRRD_L = '0ns'
    tXAW   = '16ns'
    activation_limit = 4
    addr_mapping = 'RoRaBaChCo'
    page_policy  = 'close'

    VDD    = 1.2
    IDD0   = 60
    IDD02  = 0
    IDD2N  = 26
    IDD2N2 = 0
    IDD3N  = 34
    IDD3N2 = 0
    IDD4W  = 123
    IDD4W2 = 0
    IDD4R  = 123
    IDD4R2 = 0
    IDD5   = 215
    IDD52  = 0
    IDD6   = 12
    IDD62  = 0
    IDD2P1 = 12
    IDD2P12= 0
    IDD3P1 = 20
    IDD3P12= 0
    IDD2P0 = 6
    IDD2P02= 0
    IDD3P0 = 8
    IDD3P02= 0


# ==================================================================
# HBM 2.0 pseudo-channel (64-bit bus per PC)
# ==================================================================
# Source: ETHZ-DYNAMO OptiPIM HBM2.cpp
#   https://github.com/ETHZ-DYNAMO/OptiPIM (commit 9be8f9a)
#
# HBM2 channel = 128-bit wide; each pseudo-channel is 64 bits.
# Timing parameters in ns converted from cycle counts at tCK=1ns
# (1000 MHz core clock, 2000 MT/s DDR).
#
# Organization per pseudo-channel (JEDEC JESD235B):
#   4 bank groups x 2 banks/group = 8 banks
#   1 kB row buffer
#
# Three density variants differ only in capacity and tRFC:
#   HBM2_2Gb_1x64  2 Gb per stack die -> 128 MB per PC  tRFC=160 ns
#   HBM2_4Gb_1x64  4 Gb per stack die -> 256 MB per PC  tRFC=260 ns
#   HBM2_8Gb_1x64  8 Gb per stack die -> 512 MB per PC  tRFC=350 ns
# ==================================================================

class HBM2_2Gb_1x64(DRAMInterface):
    """
    HBM 2.0 pseudo-channel, 2 Gb die (128 MB per pseudo-channel).
    Timings from ETHZ-DYNAMO OptiPIM HBM2.cpp at 2000 MT/s.
    """
    # Geometry
    device_size           = '128MB'   # 2 Gb / 2 pseudo-channels
    device_bus_width      = 64
    burst_length          = 4
    device_rowbuffer_size = '1kB'     # 1 KB per pseudo-channel (JESD235B)
    devices_per_rank      = 1
    ranks_per_channel     = 1
    bank_groups_per_rank  = 4
    banks_per_rank        = 8         # 4 BG x 2 banks/BG (JESD235B HBM2)
    channels              = 1

    # Clock: 1000 MHz core, 2000 MT/s DDR -> tCK = 1 ns
    tCK    = '1ns'

    # Burst: BL4 DDR -> 4 transfers x 0.5 ns = 2 ns
    tBURST = '2ns'

    # Core timing (OptiPIM cycles x tCK = 1 ns)
    tCL    = '7ns'    # nCL  = 7
    tRCD   = '7ns'    # nRCDRD = 7
    tRP    = '7ns'    # nRP  = 7
    tRAS   = '17ns'   # nRAS = 17
    tWR    = '8ns'    # nWR  = 8
    tRTP   = '2ns'    # nRTPS = 2

    # Bus turnaround
    tWTR   = '3ns'    # nWTRS = 3
    tRTW   = '3ns'    # nRTW  = 3
    tCS    = '1ns'    # rank-to-rank (single rank per PC, kept for completeness)

    # Bank-group CAS-to-CAS
    tCCD_L = '2ns'    # nCCDL = 2 (same bank group)

    # ACT-to-ACT
    tRRD   = '2ns'    # nRRDS = 2 (different bank group)
    tRRD_L = '3ns'    # nRRDL = 3 (same bank group)

    # Four-activation window
    tXAW   = '15ns'   # nFAW = 15
    activation_limit = 4

    # Refresh: 2 Gb die
    tRFC   = '160ns'
    tREFI  = '3900ns'  # nREFI = 3900 cycles

    addr_mapping = 'RoRaBaChCo'
    page_policy  = 'close'

    # Power (HBM2 IDD values; no published standard - use Samsung 8 Gb
    # HBM2 estimates as a placeholder, scaled for 2 Gb die)
    VDD    = 1.2
    IDD0   = 60;   IDD02  = 0
    IDD2N  = 26;   IDD2N2 = 0
    IDD3N  = 34;   IDD3N2 = 0
    IDD4W  = 123;  IDD4W2 = 0
    IDD4R  = 123;  IDD4R2 = 0
    IDD5   = 215;  IDD52  = 0
    IDD6   = 12;   IDD62  = 0
    IDD2P0 = 6;    IDD2P02= 0
    IDD2P1 = 12;   IDD2P12= 0
    IDD3P0 = 8;    IDD3P02= 0
    IDD3P1 = 20;   IDD3P12= 0


class HBM2_4Gb_1x64(HBM2_2Gb_1x64):
    """
    HBM 2.0 pseudo-channel, 4 Gb die (256 MB per pseudo-channel).
    """
    device_size = '256MB'
    tRFC        = '260ns'


class HBM2_8Gb_1x64(HBM2_2Gb_1x64):
    """
    HBM 2.0 pseudo-channel, 8 Gb die (512 MB per pseudo-channel).
    """
    device_size = '512MB'
    tRFC        = '350ns'


# Alias kept for backward compatibility with existing scripts
HBM_2000_4H_1x64 = HBM2_4Gb_1x64


# ==================================================================
# HBM 3.0 pseudo-channel (64-bit bus per pseudo-channel)
# Source: ETHZ-DYNAMO OptiPIM HBM3.cpp (commit 9be8f9a)
#
# Key architectural change vs HBM2: banks per bank group doubled.
#   HBM2: 4 BG x 2 banks/BG = 8 banks per PC  (JESD235B)
#   HBM3: 4 BG x 4 banks/BG = 16 banks per PC (JESD238)
# Same die capacity -> each bank has half the rows of HBM2.
# Timings identical to HBM2 (OptiPIM authors acknowledge TODO).
#
# RFM (Refresh Management) commands RFMab/RFMsb are new in HBM3
# as a rowhammer mitigation mechanism; not modeled here.
#
# Density variants (same total capacity as HBM2 counterparts):
#   HBM3_2Gb_1x64  2 Gb die -> 128 MB per PC  tRFC=160 ns
#   HBM3_4Gb_1x64  4 Gb die -> 256 MB per PC  tRFC=260 ns
#   HBM3_8Gb_1x64  8 Gb die -> 512 MB per PC  tRFC=350 ns
# ==================================================================

class HBM3_2Gb_1x64(HBM2_2Gb_1x64):
    """
    HBM 3.0 pseudo-channel, 2 Gb die (128 MB per pseudo-channel).
    4 BG x 4 banks/BG = 16 banks per PC. Same timings as HBM2.
    """
    banks_per_rank = 16    # 4 BG x 4 banks/BG (JESD238 HBM3)
    # device_size stays 128 MB; rows/bank = 128MB/(16*1kB) = 8192 = 1<<13


class HBM3_4Gb_1x64(HBM3_2Gb_1x64):
    """
    HBM 3.0 pseudo-channel, 4 Gb die (256 MB per pseudo-channel).
    """
    device_size = '256MB'
    tRFC        = '260ns'


class HBM3_8Gb_1x64(HBM3_2Gb_1x64):
    """
    HBM 3.0 pseudo-channel, 8 Gb die (512 MB per pseudo-channel).
    """
    device_size = '512MB'
    tRFC        = '350ns'
