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


#ifndef EMANELTE_UPLINKSINRTESTERIMPL_H
#define EMANELTE_UPLINKSINRTESTERIMPL_H

#include "sinrtesterimpl.h"
#include <map>
#include "libemanelte/otacommon.pb.h"


namespace EMANELTE {
namespace MHAL {

typedef std::pair<CHANNEL_TYPE, bool> ChannelSINRResult;
typedef std::map <CHANNEL_TYPE, bool> ChannelSINRResults;

typedef std::pair<CHANNEL_TYPE, uint16_t> ChannelRNTI;
typedef std::pair<ChannelRNTI, bool> RNTIChannelSINRResult;
typedef std::map <ChannelRNTI, bool> RNTIChannelSINRResults;


class UplinkSINRTesterImpl : public SINRTesterImpl
{
public:
  UplinkSINRTesterImpl(const float sinr, 
                       const float noiseFloor, 
                       const uint64_t carrierFrequencyHz,
                       const uint32_t carrierId) : 
   sinr_dB_{sinr},
   noiseFloor_dBm_{noiseFloor},
   carrierFrequencyHz_{carrierFrequencyHz},
   carrierId_{carrierId}
  { };
 
  ChannelSINRResults channelSINRResults_;

  RNTIChannelSINRResults rntiChannelSINRResults_;

  SINRTester::SINRTesterResult sinrCheck(const CHANNEL_TYPE ctype) override;

  SINRTester::SINRTesterResult sinrCheck(const CHANNEL_TYPE ctype, const uint16_t rnti) override;  

private:
  const float sinr_dB_;
  const float noiseFloor_dBm_;
  const uint64_t carrierFrequencyHz_;
  const uint32_t carrierId_;
};

}
}

#endif
