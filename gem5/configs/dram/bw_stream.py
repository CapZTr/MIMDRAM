# Copyright (c) 2014-2015 ARM Limited
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

"""
bw_stream.py -- streaming bandwidth test without FAW bottleneck.

Uses open-adaptive page policy so the row stays open across consecutive
bursts.  All accesses after the first ACT are pure CAS commands; FAW
never triggers.  The measured bandwidth approaches the theoretical bus
limit: burst_bytes / tBURST.

Usage:
  gem5.opt configs/dram/bw_stream.py --mem-type=HBM3_4Gb_x64
  gem5.opt configs/dram/bw_stream.py --mem-type=HBM2_4Gb_x64
  gem5.opt configs/dram/bw_stream.py --mem-type=HBM3_4Gb_x64 --banks=1
  gem5.opt configs/dram/bw_stream.py --mem-type=DDR4_2400_x64

The --banks option controls how many banks are exercised in parallel
(default: all banks).  With 1 bank the test exercises one row buffer
at a time; with N banks the generator rotates across N banks, issuing
one burst per bank per round.  Either way, stride = burst_size so every
access is a row-buffer hit (assuming open-adaptive policy keeps the row
open).

After the simulation a summary line prints:
  Bus theoretical, FAW-limited theoretical, and measured peak bandwidth.
"""

import optparse
import sys

import m5
from m5.objects import *
from m5.util import addToPath
from m5.internal.stats import periodicStatDump

addToPath('../common')

import MemConfig

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = optparse.OptionParser()

parser.add_option("--mem-type", type="choice", default="HBM3_4Gb_x64",
                  choices=MemConfig.mem_names(),
                  help="Memory type to test (default: HBM3_4Gb_x64)")

parser.add_option("--banks", type="int", default=0,
                  help="Number of banks to rotate across (0 = all banks)")

parser.add_option("--rd_perc", type="int", default=100,
                  help="Percentage of read traffic (default: 100)")

parser.add_option("--addr_map", type="int", default=1,
                  help="Address mapping: 0=RoCoRaBaCh, 1=RoRaBaCoCh (default)")

(options, args) = parser.parse_args()

if args:
    print "Error: script does not take positional arguments"
    sys.exit(1)

# ---------------------------------------------------------------------------
# System
# ---------------------------------------------------------------------------

system = System(membus=IOXBar(width=32))
system.clk_domain = SrcClockDomain(clock='2.0GHz',
                                   voltage_domain=VoltageDomain(voltage='1V'))

# 256 MB is enough; we only need a few rows worth of addresses
mem_range = AddrRange('256MB')
system.mem_ranges = [mem_range]
mmap_using_noreserve = True

options.mem_channels = 1
options.mem_ranks = 1
options.external_memory_system = 0
options.tlm_memory = 0
MemConfig.config_mem(options, system)

if not isinstance(system.mem_ctrls[0], m5.objects.DRAMCtrl):
    fatal("This script assumes the memory is a DRAMCtrl subclass")

# Force open-adaptive page policy -- the row stays open after each burst.
# This converts every subsequent same-row access from ACT+CAS+PRE to CAS-only,
# eliminating the FAW bottleneck entirely.
system.mem_ctrls[0].page_policy = "open_adaptive"
system.mem_ctrls[0].null = True

if options.addr_map == 0:
    system.mem_ctrls[0].addr_mapping = "RoCoRaBaCh"
else:
    system.mem_ctrls[0].addr_mapping = "RoRaBaCoCh"

# ---------------------------------------------------------------------------
# Geometry from the instantiated controller
# ---------------------------------------------------------------------------

ctrl = system.mem_ctrls[0]

nbr_banks  = ctrl.banks_per_rank.value
burst_size = int(ctrl.devices_per_rank.value *
                 ctrl.device_bus_width.value *
                 ctrl.burst_length.value / 8)

# Row buffer size in bytes (bytes addressable per single ACT)
page_size  = ctrl.devices_per_rank.value * ctrl.device_rowbuffer_size.value

# Number of banks the test will rotate across
test_banks = options.banks if options.banks > 0 else nbr_banks

# Issue one burst every tBURST -- the memory bus can absorb one per tBURST
# even with back-to-back CAS commands (tCCDS = 1 tCK < tBURST in all configs).
itt = ctrl.tBURST.value * 1000000000000   # ns -> ticks (1 THz global freq)

max_addr = mem_range.end

# ---------------------------------------------------------------------------
# Two measurement states
#
#   State 0: warm-up -- 1 bank, stride=burst_size, row-buffer hits
#   State 1: peak    -- test_banks banks, stride=burst_size, row-buffer hits
#
# stride = burst_size means the address pointer advances by exactly one burst
# per transaction.  As long as stride < page_size, consecutive accesses land
# in the same row buffer -> row-hit -> CAS only, no ACT, no FAW.
# ---------------------------------------------------------------------------

period = 250000000   # 250 us in ticks (1 THz) -- long enough to warm up

cfg_file_name = "configs/dram/bw_stream.cfg"
cfg_file = open(cfg_file_name, 'w')

# State 0: warm-up with 1 bank
cfg_file.write("STATE 0 %d DRAM %d 0 %d %d %d %d %d %d %d %d %d %d %d\n" % (
    period, options.rd_perc,
    max_addr, burst_size, itt, itt, 0,
    burst_size,   # stride = burst_size -> always a row hit
    page_size, nbr_banks,
    1,            # 1 bank for warm-up
    options.addr_map, 1))

# State 1: peak measurement with test_banks banks
cfg_file.write("STATE 1 %d DRAM %d 0 %d %d %d %d %d %d %d %d %d %d %d\n" % (
    period, options.rd_perc,
    max_addr, burst_size, itt, itt, 0,
    burst_size,   # stride = burst_size -> always a row hit
    page_size, nbr_banks,
    test_banks,
    options.addr_map, 1))

cfg_file.write("INIT 0\n")
cfg_file.write("TRANSITION 0 1 1\n")
cfg_file.write("TRANSITION 1 1 1\n")
cfg_file.close()

# ---------------------------------------------------------------------------
# Wire up and run
# ---------------------------------------------------------------------------

system.tgen    = TrafficGen(config_file=cfg_file_name)
system.monitor = CommMonitor()
system.tgen.port           = system.monitor.slave
system.monitor.master      = system.membus.slave
system.system_port         = system.membus.slave

periodicStatDump(period)

root = Root(full_system=False, system=system)
root.system.mem_mode = 'timing'

m5.instantiate()
m5.simulate(2 * period)   # warm-up + one measurement window

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

import re

stats_path = "m5out/stats.txt"

# gem5 periodic dumps are separated by this line
SEP = "---------- Begin Simulation Statistics ----------"

try:
    with open(stats_path) as f:
        text = f.read()
    dumps = text.split(SEP)
    # dumps[0] is before first sep (empty or header), dumps[1] = warm-up,
    # dumps[2] = measurement window (or dumps[-1] if only one dump)
    meas_dump = dumps[-1]
    m = re.search(r'system\.mem_ctrls\.bytesReadDRAM\s+(\d+)', meas_dump)
    measured_bytes = int(m.group(1)) if m else 0
except Exception:
    measured_bytes = 0

period_s  = period / 1e12          # ticks at 1 THz -> seconds
meas_bw   = measured_bytes / period_s / 1e9    # GB/s

# .value on Latency params returns SECONDS
tBURST_s  = ctrl.tBURST.value      # e.g. 2e-9 for 2 ns
tXAW_s    = ctrl.tXAW.value        # e.g. 15e-9 for 15 ns
act_limit = ctrl.activation_limit.value

# Bus-theoretical: burst_size bytes every tBURST seconds
bus_bw    = burst_size / tBURST_s / 1e9        # GB/s

# FAW limit: act_limit ACTs per tXAW -> avg 1 ACT per tXAW/act_limit
if tXAW_s > 0 and act_limit > 0:
    faw_bw  = burst_size / (tXAW_s / act_limit) / 1e9
    faw_str = "%.2f GB/s  (%d ACTs / %.0f ns window)" % (
        faw_bw, act_limit, tXAW_s * 1e9)
else:
    faw_bw  = bus_bw
    faw_str = "N/A (FAW disabled)"

print ""
print "=" * 62
print "Streaming BW test  (open-adaptive page policy, row-buffer hits)"
print "  Memory type  : %s" % options.mem_type
print "  Banks tested : %d of %d" % (test_banks, nbr_banks)
print "  Burst size   : %d B    tBURST = %.0f ns" % (burst_size, tBURST_s * 1e9)
print "  Page policy  : open_adaptive  (row stays open -> CAS-only after 1st ACT)"
print "-" * 62
print "  Bus-theoretical BW : %.2f GB/s" % bus_bw
print "  FAW-limited BW     : %s" % faw_str
print "  Measured peak BW   : %.2f GB/s  (%.1f%% of bus, %.1f%% of FAW limit)" % (
    meas_bw, meas_bw / bus_bw * 100,
    meas_bw / faw_bw * 100 if faw_bw > 0 else 0)
print "=" * 62
