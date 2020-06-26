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

#include "libemanelte/mhalenb.h"
#include "mhalenb_impl.h"


namespace {
  EMANELTE::MHAL::MHALENBImpl impl_;
}

void EMANELTE::MHAL::ENB::initialize(uint32_t idx,
                                     const EMANELTE::MHAL::mhal_config_t & mhal_config,
                                     const mhal_enb_config_t & mhal_enb_config)
{
  impl_.initialize(idx, mhal_config, mhal_enb_config);
}


void EMANELTE::MHAL::ENB::start()
{
  impl_.start();
}


void EMANELTE::MHAL::ENB::stop()
{
  impl_.stop();
}


void EMANELTE::MHAL::ENB::set_tti(uint16_t curr_tti)
{
  impl_.set_tti(curr_tti);
}


void EMANELTE::MHAL::ENB::send_msg(const EMANELTE::MHAL::Data & data, 
                                   EMANELTE::MHAL::TxControlMessage & txControl)
{
  impl_.send_msg(data, txControl);
}


bool EMANELTE::MHAL::ENB::get_messages(EMANELTE::MHAL::RxMessages & messages, timeval & tv_rx_timestamp)
{
  return impl_.get_messages(messages, tv_rx_timestamp);
}


std::uint64_t EMANELTE::MHAL::ENB::get_tx_prb_frequency(int prb_index, uint32_t carrier_id)
{
  return impl_.get_tx_prb_frequency(prb_index, carrier_id);
}
