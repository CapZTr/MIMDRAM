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

/**
 * @file
 * Shared types used between MemCtrl and DRAMInterface:
 * MemPacket (decoded memory request) and BurstHelper (split-packet tracker).
 */

#ifndef __MEM_INTERFACE_HH__
#define __MEM_INTERFACE_HH__

#include "mem/packet.hh"
#include "mem/request.hh"

/**
 * BurstHelper tracks split packets (system packets larger than one DRAM burst).
 */
struct BurstHelper
{
    const unsigned int burstCount;
    unsigned int burstsServiced;

    BurstHelper(unsigned int _burstCount)
        : burstCount(_burstCount), burstsServiced(0)
    { }
};

/**
 * MemPacket stores a memory request along with decoded address information
 * (rank, bank, row) and MIMDRAM row-operation metadata.
 *
 * It plays the same role as DRAMCtrl::DRAMPacket in the legacy controller,
 * but is shared between MemCtrl (scheduler) and DRAMInterface (device model).
 */
class MemPacket
{
  public:
    /** When the request entered the controller. */
    const Tick entryTime;

    /** When the request will be ready to send back. */
    Tick readyTime;

    /** The original system-level packet. */
    const PacketPtr pkt;

    const bool isRead;

    /** Decoded address components. */
    const uint8_t  rank;
    const uint8_t  bank;
    const uint32_t row;

    /**
     * MIMDRAM row-operation fields.
     * Only valid when is_row_op == true.
     */
    uint32_t       src1_row;
    uint32_t       src2_row;
    uint8_t        src_rank;   // for ROWCOPY: source rank
    uint8_t        src_bank;   // for ROWCOPY: source bank
    bool           is_row_op;
    Request::RowOp row_op;
    uint64_t       write_data; // for BULK_WRITE payload

    /**
     * Bank id considering all ranks: bankId = rank * banksPerRank + bank.
     * Used as index into per-bank stats vectors.
     */
    const uint16_t bankId;

    /** Starting address of this DRAM packet (may be unaligned to burst). */
    Addr addr;

    /** Size of this packet in bytes (≤ burst size). */
    unsigned int size;

    /** Non-null for split packets; tracks how many sub-bursts have been served. */
    BurstHelper* burstHelper;

    MemPacket(PacketPtr _pkt, bool is_read,
              uint8_t _rank, uint8_t _bank, uint32_t _row,
              uint16_t bank_id, Addr _addr, unsigned int _size)
        : entryTime(curTick()), readyTime(curTick()),
          pkt(_pkt), isRead(is_read),
          rank(_rank), bank(_bank), row(_row),
          src1_row(0), src2_row(0), src_rank(0), src_bank(0),
          is_row_op(false), row_op(Request::RowOp(0)), write_data(0),
          bankId(bank_id), addr(_addr), size(_size), burstHelper(NULL)
    { }
};

#endif // __MEM_INTERFACE_HH__
