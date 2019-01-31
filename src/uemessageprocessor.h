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


#ifndef EMANELTE_MODELS_UERADIOMESSAGEPROCESSOR_H
#define EMANELTE_MODELS_UERADIOMESSAGEPROCESSOR_H

#include "libemanelte/txcontrolmessage.pb.h"
#include "emane/platformserviceprovider.h"
#include "emane/frequencysegment.h"
#include "emane/utils/randomnumberdistribution.h"
#include "downlinkresourcegridparams.h"
#include "frequencysegmentbuilder.h"
#include "pormanager.h"
#include "segmentmap.h"
#include "ueradiostatisticmanager.h"
#include "uplinkresourcegridparams.h"


namespace EMANE
{
namespace Models
{
namespace LTE
{
  class UEMessageProcessor
  {
  public:
    UEMessageProcessor(EMANE::NEMId id,
                       EMANE::PlatformServiceProvider * pPlatformService,
                       UERadioStatisticManager & statisticManager) :
      id_{id},
      pPlatformService_{pPlatformService},
      statisticManager_(statisticManager),
      pUplinkRBParams_{new UplinkResourceGridParams(7)},
      downlinkMap_{},
      txFreqToRBMap_{},
      segmentBuilder_{id, pPlatformService},
      porManager_{},
      resourceBlockTxPowerdBm_{0.0f},
      RNDZeroToOne_{0.0f, 1.0f}
    {
      EMANELTE::FrequencyResourceBlockMap rbm{};
      downlinkMap_.insert(std::pair<std::uint32_t, DownlinkParams>(6, DownlinkParams(rbm, new DownlinkResourceGridParams(6, 7))));
    }

    const EMANELTE::MHAL::MESSAGE_TYPE receiveMessageType_{EMANELTE::MHAL::DOWNLINK};

    void loadPCRFile(const std::string & sPCRFileName);

    void setTxPower(float resourceBlockTxPowerdBm);

    void swapFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxFreqToRBMap,
                           EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap);

    void swapSearchFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxEvenFreqToRBMap,
                                 EMANELTE::FrequencyResourceBlockMap & rxOddFreqToRBMap,
                                 EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap);

    EMANE::FrequencySegments buildFrequencySegments(EMANELTE::MHAL::TxControlMessage & txControl,
                                                    uint32_t symbolsPerSlot);

    bool noiseTestChannelMessage(const EMANELTE::MHAL::TxControlMessage & txControl,
                                          const EMANELTE::MHAL::ChannelMessage & channel_msg,
                                          EMANE::Models::LTE::SegmentMap & segmentCache);

  private:
    struct DownlinkParams
    {
      EMANELTE::FrequencyResourceBlockMap rxFreqToMap_;
      std::unique_ptr<DownlinkResourceGridParams> pRBParams_;

      DownlinkParams(EMANELTE::FrequencyResourceBlockMap & rxFreqToMap,
                     DownlinkResourceGridParams * pRBParams)
      {
        rxFreqToMap_.swap(rxFreqToMap);
        pRBParams_.reset(pRBParams);
      }
    };

    using DownlinkMap = std::map<std::uint32_t, DownlinkParams>;

    EMANE::NEMId id_;
    EMANE::PlatformServiceProvider * pPlatformService_;
    UERadioStatisticManager & statisticManager_;
    std::unique_ptr<UplinkResourceGridParams> pUplinkRBParams_;
    DownlinkMap downlinkMap_;
    EMANELTE::FrequencyResourceBlockMap txFreqToRBMap_;
    FrequencySegmentBuilder segmentBuilder_;
    PORManager porManager_;
    float resourceBlockTxPowerdBm_;
    Utils::RandomNumberDistribution<std::mt19937,
                                    std::uniform_real_distribution<float>> RNDZeroToOne_;

    void addTxSegments(const EMANELTE::MHAL::ChannelMessage & channel_msg,
                       const std::uint32_t tti_tx);
  };

}
}
}

#endif //EMANELTE_MHAL_UERADIOMESSAGEPROCESSOR_H
