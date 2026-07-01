from m5.params import *
from m5.proxy import *
from MemObject import MemObject

class RowOpTracePlayer(MemObject):
    type = 'RowOpTracePlayer'
    cxx_header = "mem/rowop_trace_player.hh"

    port = MasterPort("Master port to send row op packets")

    trace_file = Param.String("Path to the CIMTRACE binary file")

    base_addr = Param.Addr(0, "Base address (start of channel 0)")

    # Number of banks that a single DRAMCtrl instance sees.
    # DDR4_2400_x64 : banks_per_rank(16) * ranks_per_channel(2) = 32
    # HBM2_*_x64   : banks_per_rank(8)  * ranks_per_channel(1) =  8
    # HBM3_*_x64   : banks_per_rank(16) * ranks_per_channel(1) = 16
    # The trace always uses TOTAL_BANKS=32 global bank indices; the player
    # maps global_bank to (channel = global_bank / banks_per_channel,
    #                      local_bank = global_bank % banks_per_channel).
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
    #   "simdram" : Ambit AAP/AP triple-row ops   (94_simdram_schedule_runner_hbm.c)
    #   "fcdram"  : COTS DDR4 cross-subarray gates (96_fcdram_schedule_runner_hbm.c)
    # Adding a backend is local to rowop_trace_player.cc (parseBackend + the
    # per-backend expand* implementations); see that file for the extension
    # points.  The string is validated there, with a fatal() on an unknown name.
    backend = Param.String("simdram",
        "Row-op expansion backend name (e.g. 'simdram', 'fcdram')")

    system = Param.System(Parent.any, "System this player belongs to")
