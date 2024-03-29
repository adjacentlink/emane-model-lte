/*
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * See toplevel COPYING for more information.
 */


#ifndef EMANELTE_MHAL_UE_H
#define EMANELTE_MHAL_UE_H

#include "libemanelte/mhalconfig.h"
#include "libemanelte/mhal.h"
#include "libemanelte/txcontrolmessage.pb.h"


namespace EMANELTE {
namespace MHAL {
namespace UE {

   void initialize(const uint32_t sf_interval, const EMANELTE::MHAL::mhal_config_t & mhal_config);

   void start();

   void stop();

   void set_tti(const uint16_t curr_tti);

   void send_msg(const Data & data,
                 EMANELTE::MHAL::TxControlMessage & control);

   void get_messages(RxMessages & messages,
                    timeval & rx_time);

   uint64_t get_tx_prb_frequency(const int prb_index,
                                 const uint64_t freq_hz);
  
   void set_frequencies(const uint32_t carrier_index,
                        const uint32_t pci,
                        const bool scell,
                        const uint64_t rx_frequency_hz, 
                        const uint64_t tx_frequency_hz);

   void set_num_resource_blocks(const int num_prb);

   void cell_search();
  }
 }
}


#endif
