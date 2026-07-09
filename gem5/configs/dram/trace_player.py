"""
trace_player.py  --  replay a CIMTRACE binary against a bare DRAMCtrl

CIMTRACE binary format (single, current):
  32-byte header ("CIMTRACE", version=1, record_size=48, num_records,
  last_end_time) followed by num_records x 48-byte records.  Each record
  carries a 128-bit bank bitmask (banks[2]), so the format addresses up to
  128 banks.  This is the only format the Cinnamon compiler emits.
  (A legacy 32-byte/32-bank format is still auto-detected so old traces
   replay -- no current writer produces it; see the record_size block below.)

The trace is memory-type agnostic: it names up to 128 global bank indices and
says nothing about DDR vs HBM.  Two things are chosen independently on the CLI:

  --mem-type  DRAM device/timing model.  Only sets banks_per_channel =
              banks_per_rank x ranks_per_channel; the player then instantiates
              num_channels = total_banks / banks_per_channel non-interleaved
              DRAMCtrls so all total_banks banks are addressable.  For the
              128-bank format:
                DDR4_2400_x64 : 32 banks/ch x  4 channels
                HBM3_*_x64    : 16 banks/ch x  8 channels
                HBM2_*_x64    :  8 banks/ch x 16 channels

  --backend   Row-op expansion substrate (see BACKEND_SUBARRAYS below).
                simdram : runs on DDR or HBM
                fcdram  : DDR only (COTS DDR4 cross-subarray gates)

Usage:
  gem5.opt configs/dram/trace_player.py --trace=microworkloads/trace.bin
  gem5.opt configs/dram/trace_player.py --trace=... --mem-type=HBM3_4Gb_x64
  gem5.opt configs/dram/trace_player.py --trace=... --backend=fcdram --mem-type=DDR4_2400_x64
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
                  help="DRAM type (default: DDR4_2400_x64). Independent of the "
                       "trace; banks_per_rank * ranks_per_channel must divide "
                       "the trace's total bank count (128 for the current "
                       "format). The fcdram backend requires a DDR type.")

parser.add_option("--addr-map", type="string", default="",
                  help="Override addr_mapping (e.g. RoRaBaCoCh, RoRaBaChCo). "
                       "Empty = use the memory type's default.")

# ---------------------------------------------------------------------------
# Row-op expansion backend.  Each backend lowers the same CIMTRACE schedule
# onto a different PuD substrate and needs a different number of subarrays:
#   simdram : Ambit AAP/AP, all rows in one subarray            (94_...)
#   fcdram  : compute rows in the middle subarray, reference
#             rows in BOTH neighbouring subarrays (open-bitline
#             half-row coverage needs one APA per side)         (96_...)
# To add a backend: register its name -> subarray count here and add the
# matching expand* implementation in src/mem/rowop_trace_player.cc.
# ---------------------------------------------------------------------------
BACKEND_SUBARRAYS = {
    "simdram": 1,
    "fcdram":  3,
}

parser.add_option("--backend", type="choice",
                  default="simdram",
                  choices=list(BACKEND_SUBARRAYS.keys()),
                  help="Row-op expansion backend: 'simdram' (Ambit AAP/AP, "
                       "mirrors 94_simdram_schedule_runner.c; runs on DDR or "
                       "HBM) or 'fcdram' (COTS DDR4 ROWCLONE/AND_XSUB/OR_XSUB/"
                       "NOT_XSUB/MAJ3, mirrors 96_fcdram_schedule_runner.c; "
                       "DDR only). Default: simdram.")

parser.add_option("--single-bank", action="store_true", default=False,
                  help="Single-bank optimization: an ADDI/MULI record's SIMD "
                       "op over its bank mask is a single all-bank broadcast per "
                       "channel, so it costs one bank's op-time (one "
                       "representative bank per channel is simulated) instead of "
                       "emitting every bank's packets. Same name and semantics "
                       "as OptiPIM's single_bank_opt, making the runtime "
                       "comparable to OptiPIM. Default off (full per-bank "
                       "emission).")

parser.add_option("--max-tick", type="long", default=0,
                  help="Stop the simulation after this many ticks (0 = run to "
                       "completion)")

parser.add_option("--per-channel", action="store_true", default=False,
                  help="Use the per-channel issue engine (one player port per "
                       "channel, each with its own retry slot) to remove "
                       "single-port head-of-line blocking. The start-group "
                       "dependency barrier is preserved.")

parser.add_option("--membus-width", type="int", default=32,
                  help="IOXBar membus width in bytes (default 32); raise to "
                       "probe interconnect throughput limits")

parser.add_option("--membus-clock", type="string", default="2.0GHz",
                  help="Membus/system clock (default 2.0GHz)")

(options, args) = parser.parse_args()

if not options.trace:
    print "Error: --trace is required"
    sys.exit(1)
if args:
    print "Error: unexpected positional arguments:", args
    sys.exit(1)

# ---------------------------------------------------------------------------
# Read the total bank count from the trace header's record_size field.
#   record_size == 48  ->  current format, 128-bit bank mask (up to 128 banks)
#   record_size == 32  ->  legacy format,  32-bit bank mask  (obsolete; kept
#                          only so old traces still replay -- no writer emits it)
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
if _record_size == 48:
    total_banks = 128            # current format
elif _record_size == 32:
    total_banks = 32             # legacy/obsolete format
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
# checks pass).  The FCDRAM backend needs 3 subarrays per bank (compute rows
# in the middle one, reference rows in both neighbours, one XSUB APA per
# side); SIMDRAM needs only 1.  rows_per_bank = subarrays_needed *
# ROWS_PER_SUBARRAY then stays a multiple of ROWS_PER_SUBARRAY, as DRAMCtrl
# requires.
subarrays_needed = BACKEND_SUBARRAYS[options.backend]
channel_size_min = subarrays_needed * ROWS_PER_SUBARRAY * banks_per_channel * row_stride
channel_size = 1 << int(math.ceil(math.log(channel_size_min, 2)))

base_addr = 0

# ---------------------------------------------------------------------------
# System + clock
# ---------------------------------------------------------------------------

system = System(membus=IOXBar(width=options.membus_width))
system.clk_domain = SrcClockDomain(
    clock=options.membus_clock,
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
    backend           = options.backend,
    single_bank_opt   = options.single_bank,
    per_channel       = options.per_channel)

# Single-port path is always bound (idle in per-channel mode).
system.player.port = system.membus.slave
if options.per_channel:
    # One master port per channel; the membus routes each by address to its
    # controller, and each port carries its own retry slot.
    system.player.chan_port = [system.membus.slave
                               for _ in range(num_channels)]
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
print "  Bank mode      : %s" % (
    "single-bank opt (1 rep bank/channel)" if options.single_bank
    else "full emission (per-bank packets)")
print "  Issue engine   : %s" % (
    "per-channel (independent retry/channel)" if options.per_channel
    else "single-port")
print "  Channels       :", num_channels
print "  Total banks    :", banks_per_channel * num_channels
print "  Row stride     : %d B  (DRAM row buffer size)" % row_stride
print "  Channel size   : %d MB" % (channel_size >> 20)
print "  Addr map       :", addr_map_str
print "=" * 64
print ""

exit_event = m5.simulate(options.max_tick if options.max_tick > 0
                         else m5.MaxTick)
print "Simulation exited:", exit_event.getCause(), "@ tick", m5.curTick()

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
