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

#include "libemanelte/mhalue.h"
#include "mhalue_impl.h"

namespace {
  EMANELTE::MHAL::MHALUEImpl impl_;
}


void EMANELTE::MHAL::UE::initialize(const uint32_t sf_interval_msec,
                                    const EMANELTE::MHAL::mhal_config_t & mhal_config)
{
  impl_.initialize(sf_interval_msec, mhal_config);
}


void EMANELTE::MHAL::UE::start()
{
  impl_.start();
}


void EMANELTE::MHAL::UE::stop()
{
  impl_.stop();
}


void EMANELTE::MHAL::UE::set_tti(const uint16_t curr_tti)
{
  impl_.set_tti(curr_tti);
}


void EMANELTE::MHAL::UE::send_msg(const EMANELTE::MHAL::Data & data, 
                                  EMANELTE::MHAL::TxControlMessage & tx_control)
{
  impl_.send_msg(data, tx_control);
}


void
EMANELTE::MHAL::UE::get_messages(EMANELTE::MHAL::RxMessages & messages, timeval & tv_rx_timestamp)
{
  impl_.get_messages(messages, tv_rx_timestamp);
}


uint64_t
EMANELTE::MHAL::UE::get_tx_prb_frequency(const int prb_index, const uint64_t frequency_hz)
{
  return impl_.get_tx_prb_frequency(prb_index, frequency_hz);
}


void EMANELTE::MHAL::UE::set_frequencies(const uint32_t carrier_index,
                                         const uint32_t pci,
                                         const bool scell,
                                         const uint64_t rx_frequency_hz,
                                         const uint64_t tx_frequency_hz)
{
  impl_.set_frequencies(carrier_index, pci, scell, rx_frequency_hz, tx_frequency_hz);
}


void EMANELTE::MHAL::UE::set_num_resource_blocks(const int num_prb)
{
  impl_.set_num_resource_blocks(num_prb);
}


void EMANELTE::MHAL::UE::cell_search()
{
  impl_.cell_search();
}
