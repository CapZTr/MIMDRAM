"""
trace_player.py  --  replay a CIMTRACE binary against a bare DRAMCtrl

Supported memory types and how they map to 32 MIMDRAM global banks:

  DDR4_2400_x64  : banks_per_rank=16 ranks=2 -> 32 banks/ch x 1 channel
  HBM3_*_x64    : banks_per_rank=16 ranks=1 -> 16 banks/ch x 2 channels
  HBM2_*_x64    : banks_per_rank=8  ranks=1 ->  8 banks/ch x 4 channels

For multi-channel configs the channels are given non-overlapping address
ranges (no XOR-hashed interleaving).  The trace player maps

  global_bank in [0,32)  ->  channel    = global_bank / banks_per_channel
                              local_bank = global_bank % banks_per_channel

Usage:
  gem5.opt configs/dram/trace_player.py --trace=microworkloads/trace.bin
  gem5.opt configs/dram/trace_player.py --trace=... --mem-type=HBM2_4Gb_x64
  gem5.opt configs/dram/trace_player.py --trace=... --mem-type=HBM3_4Gb_x64
"""

import math
import optparse
import sys
import re

import m5
from m5.objects import *
from m5.util import addToPath

addToPath('../common')
import MemConfig

# ---------------------------------------------------------------------------
# Constants (must match mimdram.h and request.hh)
# ---------------------------------------------------------------------------
TOTAL_BANKS       = 32      # BANK_COUNT(16) * RANK_COUNT(2)
ROWS_PER_SUBARRAY = 512     # rows_per_subarray from mimdram.h
ROW_SIZE_BYTES    = 8192    # #define ROW_SIZE in request.hh

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = optparse.OptionParser()

parser.add_option("--trace", type="string", default="",
                  help="Path to the CIMTRACE binary file (required)")

parser.add_option("--mem-type", type="choice",
                  default="DDR4_2400_x64",
                  choices=MemConfig.mem_names(),
                  help="DRAM type (default: DDR4_2400_x64). "
                       "banks_per_rank * ranks_per_channel must divide 32.")

parser.add_option("--addr-map", type="string", default="",
                  help="Override addr_mapping (e.g. RoRaBaCoCh, RoRaBaChCo). "
                       "Empty = use the memory type's default.")

(options, args) = parser.parse_args()

if not options.trace:
    print "Error: --trace is required"
    sys.exit(1)
if args:
    print "Error: unexpected positional arguments:", args
    sys.exit(1)

# ---------------------------------------------------------------------------
# Memory geometry
# ---------------------------------------------------------------------------

mem_cls = MemConfig.get(options.mem_type)
tmp     = mem_cls()

banks_per_channel = int(tmp.banks_per_rank.value) * \
                    int(tmp.ranks_per_channel.value)

if TOTAL_BANKS % banks_per_channel != 0:
    fatal("'%s' has %d banks/channel (%d banks/rank x %d ranks), which does "
          "not divide TOTAL_BANKS=%d.  Choose a type whose banks_per_rank x "
          "ranks_per_channel is 8, 16, or 32." %
          (options.mem_type, banks_per_channel,
           int(tmp.banks_per_rank.value), int(tmp.ranks_per_channel.value),
           TOTAL_BANKS))

num_channels = TOTAL_BANKS // banks_per_channel

# Per-channel address space: enough for ROWS_PER_SUBARRAY rows of all local
# banks.  Rounded up to the next power of 2 so DRAMCtrl geometry checks pass.
channel_size_min = ROWS_PER_SUBARRAY * banks_per_channel * ROW_SIZE_BYTES
channel_size = 1 << int(math.ceil(math.log(channel_size_min, 2)))

base_addr = 0

# ---------------------------------------------------------------------------
# System + clock
# ---------------------------------------------------------------------------

system = System(membus=IOXBar(width=32))
system.clk_domain = SrcClockDomain(
    clock='2.0GHz',
    voltage_domain=VoltageDomain(voltage='1V'))

system.mem_ranges = [AddrRange(base_addr,
                               size=num_channels * channel_size)]

# ---------------------------------------------------------------------------
# DRAMCtrl instances - one per channel, non-overlapping address ranges.
# Using standalone (non-interleaved) ranges avoids the XOR hash that
# MemConfig injects for multi-channel configs, keeping bank->channel
# mapping fully deterministic.
# ---------------------------------------------------------------------------

ctrls = []
for c in range(num_channels):
    ctrl = mem_cls()
    ctrl.channels = 1   # standalone: no interleaving metadata
    if options.addr_map:
        ctrl.addr_mapping = options.addr_map
    ctrl.range = AddrRange(base_addr + c * channel_size, size=channel_size)
    ctrls.append(ctrl)

system.mem_ctrls = ctrls

for ctrl in system.mem_ctrls:
    ctrl.port = system.membus.master

# ---------------------------------------------------------------------------
# RowOpTracePlayer
# ---------------------------------------------------------------------------

system.player = RowOpTracePlayer(
    trace_file        = options.trace,
    base_addr         = base_addr,
    banks_per_channel = banks_per_channel,
    channel_size      = channel_size)

system.player.port = system.membus.slave
system.system_port = system.membus.slave

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

root = Root(full_system=False, system=system)
root.system.mem_mode = 'timing'

m5.instantiate()

addr_map_str = options.addr_map if options.addr_map else str(tmp.addr_mapping)
print ""
print "=" * 64
print "RowOp trace player"
print "  Trace          :", options.trace
print "  Memory type    :", options.mem_type
print "  Banks/channel  : %d  (%d banks/rank x %d ranks)" % (
    banks_per_channel,
    int(tmp.banks_per_rank.value),
    int(tmp.ranks_per_channel.value))
print "  Channels       :", num_channels
print "  Total banks    :", banks_per_channel * num_channels
print "  Channel size   : %d MB" % (channel_size >> 20)
print "  Addr map       :", addr_map_str
print "=" * 64
print ""

exit_event = m5.simulate()
print "Simulation exited:", exit_event.getCause()

m5.stats.dump()

# ---------------------------------------------------------------------------
# Stats summary
# ---------------------------------------------------------------------------

stats_path = m5.options.outdir + "/stats.txt"
try:
    with open(stats_path) as f:
        text = f.read()

    def extract(pat, txt):
        m = re.search(pat, txt)
        return m.group(1) if m else "N/A"

    pkts    = extract(r'system\.player\.numPacketsSent\s+(\S+)', text)
    retries = extract(r'system\.player\.numRetries\s+(\S+)', text)
    wr_reqs = extract(r'system\.mem_ctrls\.writeReqs\s+(\S+)', text)

    # Sum ACT ticks across all channel controllers
    act_total = 0
    act_found = False
    for m in re.finditer(
            r'system\.mem_ctrls[^.]*\.memoryStateTime::ACT\s+(\d+)', text):
        act_total += int(m.group(1))
        act_found = True
    act_str = str(act_total) if act_found else "N/A"

    print ""
    print "=" * 64
    print "Results"
    print "  Packets sent          :", pkts
    print "  Retries (back-press)  :", retries
    print "  DRAM write reqs (ch0) :", wr_reqs
    print "  Total DRAM ACT ticks  :", act_str
    print "=" * 64
except IOError:
    print "(stats file not found at %s)" % stats_path
