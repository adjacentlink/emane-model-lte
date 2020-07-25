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


#include "uplinksinrtesterimpl.h"



EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::UplinkSINRTesterImpl::sinrCheck2(CHANNEL_TYPE ctype, uint64_t carrierFrequencyHz)
{
  auto sinr_result = channelSINRResults_.find(ctype);

  if(sinr_result == channelSINRResults_.end())
    {
      return SINRTester::SINRTesterResult{};
    }

  return SINRTester::SINRTesterResult{sinr_result->second, sinr_dB_, noiseFloor_dBm_};
}


EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::UplinkSINRTesterImpl::sinrCheck2(CHANNEL_TYPE ctype, uint16_t rnti, uint64_t carrierFrequencyHz)
{
  auto sinr_result = rntiChannelSINRResults_.find(EMANELTE::MHAL::ChannelRNTI(ctype, rnti));

  if(sinr_result == rntiChannelSINRResults_.end())
    {
      return SINRTester::SINRTesterResult{};
    }

  return SINRTester::SINRTesterResult{sinr_result->second, sinr_dB_, noiseFloor_dBm_};
}
