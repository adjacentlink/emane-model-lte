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


#ifndef EMANELTE_MHAL_ENB_H
#define EMANELTE_MHAL_ENB_H

#include "libemanelte/mhalconfig.h"
#include "libemanelte/mhal.h"
#include "libemanelte/txcontrolmessage.pb.h"


namespace EMANELTE {
namespace MHAL {
namespace ENB {

  struct mhal_enb_config_t
  {
    uint32_t physical_cell_id_;
    uint32_t subframe_interval_msec_;
    uint32_t symbols_per_slot_;
    double uplink_frequency_hz_;
    double downlink_frequency_hz_;
    int num_resource_blocks_;
    float pdsch_rs_power_milliwatt_;
    float pdsch_rho_b_over_rho_a_;

    mhal_enb_config_t(uint32_t physical_cell_id,
                      uint32_t subframe_interval_msec,
                      uint32_t symbols_per_slot,
                      double uplink_frequency_hz,
                      double downlink_frequency_hz,
                      int num_resource_blocks,
                      float pdsch_rs_power_milliwatt,
                      float pdsch_rho_b_over_rho_a) :
      physical_cell_id_(physical_cell_id),
      subframe_interval_msec_(subframe_interval_msec),
      symbols_per_slot_(symbols_per_slot),
      uplink_frequency_hz_(uplink_frequency_hz),
      downlink_frequency_hz_(downlink_frequency_hz),
      num_resource_blocks_(num_resource_blocks),
      pdsch_rs_power_milliwatt_(pdsch_rs_power_milliwatt),
      pdsch_rho_b_over_rho_a_(pdsch_rho_b_over_rho_a)
    {}
  };

  void initialize(uint32_t idx,
                  const EMANELTE::MHAL::mhal_config_t & mhal_config,
                  const mhal_enb_config_t & mhal_enb_config);

  void start();

  void stop();

  void set_tti(uint16_t curr_tti);

  void send_msg(const Data & data, EMANELTE::MHAL::TxControlMessage & control);

  bool get_messages(RxMessages & messages, timeval & rx_time);

  std::uint64_t get_tx_prb_frequency(int prb_index, uint64_t freq_hz);

}
}
}

#endif
