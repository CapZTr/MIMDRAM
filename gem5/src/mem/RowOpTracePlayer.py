from m5.params import *
from m5.proxy import *
from MemObject import MemObject

class RowOpTracePlayer(MemObject):
    type = 'RowOpTracePlayer'
    cxx_header = "mem/rowop_trace_player.hh"

    port = MasterPort("Master port to send row op packets")

    # Per-channel issue path: one master port per channel, each with its own
    # retry slot, so back-pressure on one channel does not stall issues to the
    # others (removes the single-port head-of-line blocking).  Left unconnected
    # in the default single-port mode.  The start-group dependency barrier is
    # preserved in this mode; only within-group cross-channel head-of-line
    # blocking is removed.
    chan_port = VectorMasterPort("Per-channel master ports")
    per_channel = Param.Bool(False,
        "Use the per-channel issue engine (one port + retry slot per channel); "
        "False = legacy single-port engine")

    trace_file = Param.String("Path to the CIMTRACE binary file")

    base_addr = Param.Addr(0, "Base address (start of channel 0)")

    # Number of banks that a single DRAMCtrl instance sees.
    # DDR4_2400_x64 : banks_per_rank(16) * ranks_per_channel(2) = 32
    # HBM2_*_x64   : banks_per_rank(8)  * ranks_per_channel(1) =  8
    # HBM3_*_x64   : banks_per_rank(16) * ranks_per_channel(1) = 16
    # The trace uses up to 128 global bank indices (128-bit bank mask); the
    # player maps global_bank to (channel = global_bank / banks_per_channel,
    #                             local_bank = global_bank % banks_per_channel).
    banks_per_channel = Param.Int(32, "Banks per DRAMCtrl instance")

    # Byte distance between consecutive channel base addresses.
    # 0 is valid only when banks_per_channel == 32 (single channel).
    channel_size = Param.Addr(0, "Address range per channel (bytes)")

    # DRAM row buffer size (device_rowbuffer_size from the timing config).
    # Used as the address stride between consecutive slot rows so that
    # DRAMCtrl's bank/row decode assigns the correct bank to each address.
    # For DDR4 this equals ROW_SIZE (8192); for HBM2/HBM3 it is 1024.
    row_stride = Param.Addr(8192, "DRAM row buffer size in bytes (row stride)")

    # Row-op expansion backend: selects which PuD substrate the same CIMTRACE
    # schedule is lowered onto.  Each backend mirrors one schedule-runner
    # microworkload and emits a different DRAM row-op sequence:
    #   "simdram" : Ambit AAP/AP triple-row ops   (94_simdram_schedule_runner.c;
    #               runs on DDR or HBM)
    #   "fcdram"  : COTS DDR4 cross-subarray gates (96_fcdram_schedule_runner.c;
    #               DDR only)
    #   "prada"   : SRA TRA/N/5RA row-ops          (95_prada_schedule_runner.c;
    #               runs on DDR or HBM)
    # Adding a backend is local to rowop_trace_player.cc (parseBackend + the
    # per-backend expand* implementations); see that file for the extension
    # points.  The string is validated there, with a fatal() on an unknown name.
    backend = Param.String("simdram",
        "Row-op expansion backend name (e.g. 'simdram', 'fcdram')")

    # Single-bank optimization.  An ADDI/MULI record runs the SAME row-op on
    # the SAME rows across every bank in its 128-bit mask (SIMD); real PuD
    # hardware issues that as ONE all-bank broadcast command per channel, so it
    # costs one bank's op-time, not one per bank.  When True the player keeps a
    # single representative bank per channel for those records, so bank count
    # no longer inflates the makespan (channels are independent DRAMCtrls and
    # stay parallel).  Same name and semantics as OptiPIM's single_bank_opt,
    # which is what makes the gem5 row-op runtime comparable to OptiPIM.
    # Default False = full emission: every bank in the mask gets its own
    # packets.  ROWCOPY is unaffected either way: it addresses specific
    # src/dst banks, not the mask.
    single_bank_opt = Param.Bool(False,
        "Collapse each ADDI/MULI record to one representative bank per "
        "channel (all-bank broadcast credit, mirrors OptiPIM's "
        "single_bank_opt); False = full per-bank emission")

    system = Param.System(Parent.any, "System this player belongs to")
