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

    system = Param.System(Parent.any, "System this player belongs to")
