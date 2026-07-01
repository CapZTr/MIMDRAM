"""
trace_player.py  --  replay a CIMTRACE binary against a bare DRAMCtrl

Two CIMTRACE binary formats are supported, auto-detected from the header:

  record_size==32: 32-bank format  (93_simdram_schedule_runner.c)
  record_size==48: 128-bank format (94_simdram_schedule_runner_hbm.c)

Memory type to channel mapping:

  32-bank traces:
    DDR4_2400_x64 : 32 banks/ch x 1 channel
    HBM3_*_x64   : 16 banks/ch x 2 channels
    HBM2_*_x64   :  8 banks/ch x 4 channels

  128-bank traces:
    HBM3_*_x64   : 16 banks/ch x 8 channels
    HBM2_*_x64   :  8 banks/ch x 16 channels

Usage:
  gem5.opt configs/dram/trace_player.py --trace=microworkloads/trace.bin
  gem5.opt configs/dram/trace_player.py --trace=... --mem-type=HBM2_4Gb_x64
  gem5.opt configs/dram/trace_player.py --trace=... --mem-type=HBM3_4Gb_x64
"""

import math
import optparse
import struct
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
                       "banks_per_rank * ranks_per_channel must divide "
                       "the trace total_banks (32 or 128).")

parser.add_option("--addr-map", type="string", default="",
                  help="Override addr_mapping (e.g. RoRaBaCoCh, RoRaBaChCo). "
                       "Empty = use the memory type's default.")

# ---------------------------------------------------------------------------
# Row-op expansion backend.  Each backend lowers the same CIMTRACE schedule
# onto a different PuD substrate and needs a different number of subarrays:
#   simdram : Ambit AAP/AP, all rows in one subarray            (94_...)
#   fcdram  : COTS cross-subarray gates need com + ref rows in
#             two neighbouring subarrays                        (96_...)
# To add a backend: register its name -> subarray count here and add the
# matching expand* implementation in src/mem/rowop_trace_player.cc.
# ---------------------------------------------------------------------------
BACKEND_SUBARRAYS = {
    "simdram": 1,
    "fcdram":  2,
}

parser.add_option("--backend", type="choice",
                  default="simdram",
                  choices=list(BACKEND_SUBARRAYS.keys()),
                  help="Row-op expansion backend: 'simdram' (Ambit AAP/AP, "
                       "mirrors 94_simdram_schedule_runner_hbm.c) or 'fcdram' "
                       "(COTS DDR4 ROWCLONE/AND_XSUB/OR_XSUB/NOT_XSUB, mirrors "
                       "96_fcdram_schedule_runner_hbm.c). Default: simdram.")

(options, args) = parser.parse_args()

if not options.trace:
    print "Error: --trace is required"
    sys.exit(1)
if args:
    print "Error: unexpected positional arguments:", args
    sys.exit(1)

# ---------------------------------------------------------------------------
# Auto-detect total bank count from the trace header.
#   record_size == 32  ->  32-bank format  (uint32_t banks bitmask)
#   record_size == 48  ->  128-bank format (uint64_t banks[2] bitmask)
# ---------------------------------------------------------------------------

try:
    with open(options.trace, 'rb') as _tf:
        _hdr = _tf.read(32)
except IOError as e:
    print "Error: cannot open trace file '%s': %s" % (options.trace, e)
    sys.exit(1)

if len(_hdr) < 32 or _hdr[:8] != 'CIMTRACE':
    print "Error: '%s' is not a valid CIMTRACE file" % options.trace
    sys.exit(1)

_record_size = struct.unpack_from('<I', _hdr, 12)[0]
if _record_size == 32:
    total_banks = 32
elif _record_size == 48:
    total_banks = 128
else:
    print "Error: unrecognised record_size=%d in '%s'" % (_record_size, options.trace)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Memory geometry
# ---------------------------------------------------------------------------

mem_cls = MemConfig.get(options.mem_type)
tmp     = mem_cls()

banks_per_channel = int(tmp.banks_per_rank.value) * \
                    int(tmp.ranks_per_channel.value)

if total_banks % banks_per_channel != 0:
    fatal("'%s' has %d banks/channel (%d banks/rank x %d ranks), which does "
          "not divide total_banks=%d (from trace).  Choose a type whose "
          "banks_per_rank x ranks_per_channel divides %d." %
          (options.mem_type, banks_per_channel,
           int(tmp.banks_per_rank.value), int(tmp.ranks_per_channel.value),
           total_banks, total_banks))

num_channels = total_banks // banks_per_channel

# row_stride: the effective DRAM row buffer size per rank (= device_rowbuffer_size
# * devices_per_rank) determines the address stride between consecutive slot rows
# in slotAddr().  Using this (not application ROW_SIZE) ensures DRAMCtrl's
# bank/row decode assigns the correct DRAM bank to each address and keeps all
# slot rows within a single 512-row subarray.
#   DDR4:       1 kB/dev x 8 devs/rank = 8 kB  (= ROW_SIZE, no change vs before)
#   HBM2/HBM3: 1 kB/dev x 1 dev/rank  = 1 kB
row_stride = (int(tmp.device_rowbuffer_size.getValue()) *
              int(tmp.devices_per_rank.value))

# Per-channel address space: subarrays_needed * ROWS_PER_SUBARRAY row-buffer
# rows per local bank (rounded up to the next power of 2 so DRAMCtrl geometry
# checks pass).  The FCDRAM backend needs 2 neighbouring subarrays per bank so
# a compute row and its cross-subarray reference row are both addressable;
# SIMDRAM needs only 1.  rows_per_bank = subarrays_needed * ROWS_PER_SUBARRAY
# then stays a multiple of ROWS_PER_SUBARRAY, as DRAMCtrl requires.
subarrays_needed = BACKEND_SUBARRAYS[options.backend]
channel_size_min = subarrays_needed * ROWS_PER_SUBARRAY * banks_per_channel * row_stride
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
    channel_size      = channel_size,
    row_stride        = row_stride,
    backend           = options.backend)

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
print "  Trace format   : %d-bank" % total_banks
print "  Backend        : %s  (%d subarray(s)/bank)" % (
    options.backend, subarrays_needed)
print "  Channels       :", num_channels
print "  Total banks    :", banks_per_channel * num_channels
print "  Row stride     : %d B  (DRAM row buffer size)" % row_stride
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

    pkts     = extract(r'system\.player\.numPacketsSent\s+(\S+)', text)
    retries  = extract(r'system\.player\.numRetries\s+(\S+)', text)
    makespan = extract(r'system\.player\.rowOpMakespan\s+(\S+)', text)
    wr_reqs  = extract(r'system\.mem_ctrls\.writeReqs\s+(\S+)', text)

    # Sum ACT ticks across all channel controllers
    act_total = 0
    act_found = False
    for m in re.finditer(
            r'system\.mem_ctrls[^.]*\.memoryStateTime::ACT\s+(\d+)', text):
        act_total += int(m.group(1))/1000
        act_found = True
    act_str = str(act_total) if act_found else "N/A"

    # Row-op runtime: makespan of the row-op phase (benchmark wall-clock).
    # sim_freq is 1e12 ticks/s, so 1 ns == 1000 ticks.
    try:
        runtime_ns = "%.1f" % (int(makespan) / 1000.0)
    except (ValueError, TypeError):
        runtime_ns = "N/A"

    print ""
    print "=" * 64
    print "Results"
    print "  Row-op runtime (ns)   :", runtime_ns, "   <-- benchmark runtime"
    print "  Row-op runtime (ticks):", makespan
    print "  Packets sent          :", pkts
    print "  Retries (back-press)  :", retries
    print "  DRAM write reqs (ch0) :", wr_reqs
    print "  Total DRAM ACT ticks  :", act_str, "  (aggregate active time, NOT runtime)"
    print "=" * 64
except IOError:
    print "(stats file not found at %s)" % stats_path
