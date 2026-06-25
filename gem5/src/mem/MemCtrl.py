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
from AbstractMemory import *
from DRAMCtrl import MemSched

class MemCtrl(AbstractMemory):
    type       = 'MemCtrl'
    cxx_header = 'mem/mem_ctrl.hh'

    # DRAM device interface
    dram = Param.DRAMInterface('DRAM device interface')

    # Read / write buffer sizes (in bursts)
    read_buffer_size  = Param.Unsigned(32, 'Read queue depth')
    write_buffer_size = Param.Unsigned(64, 'Write queue depth')

    # Write-drain thresholds (% of write_buffer_size)
    write_high_thresh_perc = Param.Percent(85,
        'Trigger write drain when write queue fills to this fraction')
    write_low_thresh_perc  = Param.Percent(50,
        'Return to reads when write queue drains below this fraction')
    min_writes_per_switch  = Param.Unsigned(16,
        'Minimum writes to issue per write-drain episode')

    # FR-FCFS vs FCFS
    mem_sched_policy = Param.MemSched('frfcfs', 'Memory scheduling policy')

    # Controller pipeline latencies
    static_frontend_latency = Param.Latency('10ns',
        'Constant latency from port to controller')
    static_backend_latency  = Param.Latency('10ns',
        'Constant latency from controller to DRAM data return')

    # Slave port
    port = SlavePort('CPU-side port')
