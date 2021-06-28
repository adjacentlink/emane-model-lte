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

#ifndef EMANELTE_MHALENB_IMPL_H
#define EMANELTE_MHALENB_IMPL_H

#include "mhalcommon.h"
#include "uplinksinrtesterimpl.h"
#include "libemanelte/mhalenb.h"

namespace EMANELTE {
namespace MHAL {

  class MHALENBImpl : public MHALCommon
  {
  public:
    MHALENBImpl() :
      MHALCommon(),
      nems_{},
      pRadioModel_{NULL}
    {}

    void initialize(uint32_t idx,
                    const mhal_config_t & mhal_config,
                    const ENB::mhal_enb_config_t & mhal_enb_config);

    void init_emane();

    void start();

    void send_downstream(const Data & data,
                         TxControlMessage & txControl,
                         const EMANE::TimePoint & timestamp);

    void handle_upstream_msg(const Data & data,
                             const RxControl & rxControl,
                             const PHY::OTAInfo & otaInfo,
                             const TxControlMessage & txControl);

    EMANE::SpectrumWindow get_noise(const uint32_t antennaIndex,
                                    const FrequencyHz frequencyHz,
                                    const EMANE::Microseconds & span,
                                    const EMANE::TimePoint & sor);

    std::uint64_t get_tx_prb_frequency(int prb_index, std::uint64_t freq_hz);

    void noise_processor(const uint32_t bin, const EMANE::Models::LTE::AntennaSpectrumWindowCache & antennaSpectrumWindowCache);

  private:
    std::set<std::uint32_t>   physicalCellIds_;
    EMANE::Application::NEMs  nems_;
    std::unique_ptr<EMANE::Application::NEMManager> pNEMManager_;
    EMANE::Models::LTE::ENBRadioModel * pRadioModel_;

    void putSINRResult_i(const ChannelMessage & channel_message,
                         const RxControl & rxControl,
                         UplinkSINRTesterImpl * pSINRTester,
                         const bool received);

    bool checkPci_i(const TxControlMessage & txControl) const;
  };
}
}

#endif

