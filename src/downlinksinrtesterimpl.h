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


#ifndef EMANELTE_DOWNLINKSINRTESTERIMPL_H
#define EMANELTE_DOWNLINKSINRTESTERIMPL_H

#include "sinrtesterimpl.h"
#include "radiomodel.h"


namespace EMANELTE {
namespace MHAL {

class DownlinkSINRTesterImpl : public SINRTesterImpl
{
public:
  DownlinkSINRTesterImpl(EMANE::Models::LTE::UERadioModel * pRadioModel,
                         TxControlMessage txControl,
                         EMANE::Models::LTE::SegmentMap segmentCache,
                         bool pcfichPass,
                         bool pbchPass,
                         double sinr,
                         double noiseFloor) :
    pRadioModel_{pRadioModel},
    txControl_{txControl},
    segmentCache_{segmentCache},
    pcfichPass_{pcfichPass},
    pbchPass_{pbchPass},
    sinr_dB_{sinr},
    noiseFloor_dBm_{noiseFloor},
    pdcchRNTIResults_{}
  {}


  SINRTester::SINRTesterResult sinrCheck2(CHANNEL_TYPE ctype, uint64_t rx_freq_hz) override;

  SINRTester::SINRTesterResult sinrCheck2(CHANNEL_TYPE ctype, uint16_t rnti, uint64_t rx_freq_hz) override;  

private:
  EMANE::Models::LTE::UERadioModel * const pRadioModel_;
  const TxControlMessage txControl_;
  EMANE::Models::LTE::SegmentMap segmentCache_;
  const bool pcfichPass_;
  const bool pbchPass_;
  const double sinr_dB_;
  const double noiseFloor_dBm_;
  std::map<std::uint32_t, bool> pdcchRNTIResults_;
};

}
}

#endif
