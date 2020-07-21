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


#include "downlinksinrtesterimpl.h"



EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::DownlinkSINRTesterImpl::sinrCheck2(CHANNEL_TYPE ctype, uint64_t rx_freq_hz)
{
  if(ctype == CHAN_PCFICH)
    {
      return SINRTester::SINRTesterResult{pcfichPass_, sinr_dB_, noiseFloor_dBm_};
    }
  else if(ctype == CHAN_PBCH)
    {
      return SINRTester::SINRTesterResult{pbchPass_, sinr_dB_, noiseFloor_dBm_};
    }
  else if(ctype == CHAN_PMCH)
    {
      auto carrier = txControl_.carriers().find(rx_freq_hz);

      if(carrier != txControl_.carriers().end() && carrier->second.downlink().has_pmch())
        {
          return SINRTester::SINRTesterResult{
                   pRadioModel_->noiseTestChannelMessage(txControl_, 
                                                         carrier->second.downlink().pmch(),
                                                         segmentCache_,
                                                         carrier->first),
                   sinr_dB_,
                   noiseFloor_dBm_};
        }
    }

  return SINRTester::SINRTesterResult{};
}


EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::DownlinkSINRTesterImpl::sinrCheck2(CHANNEL_TYPE ctype, uint16_t rnti, uint64_t rx_freq_hz)
{
  if(!pcfichPass_)
    {
      // fail to decode PCFICH means nothing received
      return SINRTester::SINRTesterResult{};
    }

  // For other physical channels, match the specified rnti
  if(ctype == CHAN_PHICH)
    {
      auto carrier = txControl_.carriers().find(rx_freq_hz);

      if(carrier != txControl_.carriers().end())
        {
          for(int i = 0; i < carrier->second.downlink().phich_size(); ++i)
            {
              if(carrier->second.downlink().phich(i).rnti() != rnti)
               {
                 continue;
               }

             return SINRTester::SINRTesterResult{
                      pRadioModel_->noiseTestChannelMessage(txControl_,
                                                            carrier->second.downlink().phich(i),
                                                            segmentCache_,
                                                            carrier->first),
                      sinr_dB_,
                      noiseFloor_dBm_};
           }
        }
    }
  else if(ctype == CHAN_PDCCH)
    {
      auto carrier = txControl_.carriers().find(rx_freq_hz);

      if(carrier != txControl_.carriers().end())
        {
          for(int i = 0; i < carrier->second.downlink().pdcch_size(); ++i)
            {
              if(carrier->second.downlink().pdcch(i).rnti() != rnti)
                {
                  continue;
                }

              bool pdcchPass{pRadioModel_->noiseTestChannelMessage(txControl_,
                                                                   carrier->second.downlink().pdcch(i),
                                                                   segmentCache_,
                                                                   carrier->first)};

              // Store PDCCH result for corresponding PDSCH check
              pdcchRNTIResults_.emplace(rnti, pdcchPass);

              return SINRTester::SINRTesterResult{pdcchPass, sinr_dB_, noiseFloor_dBm_};
            }
        }
    }
  else if(ctype == CHAN_PDSCH)
    {
      auto pdcchIter = pdcchRNTIResults_.find(rnti);

      // PDSCH reception only if corresponding PDCCH was received
      if(pdcchIter == pdcchRNTIResults_.end() || !pdcchIter->second)
        {
          return SINRTester::SINRTesterResult{};
        }

      auto carrier = txControl_.carriers().find(rx_freq_hz);

      if(carrier != txControl_.carriers().end())
        {
          for(int i = 0; i < carrier->second.downlink().pdsch_size(); ++i)
            {
              if(carrier->second.downlink().pdsch(i).rnti() != rnti)
                {
                  continue;
                }

              return SINRTester::SINRTesterResult{pRadioModel_->noiseTestChannelMessage(txControl_, 
                                                                                        carrier->second.downlink().pdsch(i),
                                                                                        segmentCache_,
                                                                                        carrier->first), 
                                                  sinr_dB_,
                                                  noiseFloor_dBm_};
            }
        }
    }
  else
    {
      // unexpected channel
    }

  return SINRTester::SINRTesterResult{};
}
