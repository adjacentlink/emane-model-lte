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


#ifndef EMANELTE_MHAL_ENBRADIOMESSAGEPROCESSOR_H
#define EMANELTE_MHAL_ENBRADIOMESSAGEPROCESSOR_H

#include "libemanelte/txcontrolmessage.pb.h"
#include "emane/platformserviceprovider.h"
#include "emane/frequencysegment.h"
#include "emane/utils/randomnumberdistribution.h"
#include "downlinkresourcegridparams.h"
#include "enbradiostatisticmanager.h"
#include "frequencysegmentbuilder.h"
#include "pormanager.h"
#include "segmentmap.h"
#include "uplinkresourcegridparams.h"


namespace EMANE
{
namespace Models
{
namespace LTE
{
  class ENBMessageProcessor
  {
  public:
    ENBMessageProcessor(EMANE::NEMId id,
                        EMANE::PlatformServiceProvider * pPlatformService,
                        ENBRadioStatisticManager & statisticManager) :
      id_{id},
      pPlatformService_{pPlatformService},
      statisticManager_(statisticManager),
      pDownlinkRBParams_{new DownlinkResourceGridParams(6, 7)},
      pUplinkRBParams_{new UplinkResourceGridParams(7)},
      rxFreqToRBMap_{},
      txFreqToRBMap_{},
      segmentBuilder_{id, pPlatformService},
      porManager_{},
      resourceBlockTxPowerdBm_{0.0f},
      RNDZeroToOne_{0.0f, 1.0f}
    {}

    const EMANELTE::MHAL::MESSAGE_TYPE receiveMessageType_{EMANELTE::MHAL::UPLINK};

    void loadPCRFile(const std::string & sPCRFileName);

    void setTxPower(float resourceBlockTxPowerdBm);

    void swapFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxFreqToRBMap,
                           EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap);

    void swapSearchFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxEvenFreqToRBMap,
                                 EMANELTE::FrequencyResourceBlockMap & rxOddFreqToRBMap,
                                 EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap);

    EMANE::FrequencySegments buildFrequencySegments(EMANELTE::MHAL::TxControlMessage & txControl,
                                                    const uint64_t carrierFreqHz,
                                                    const uint32_t carrierId);

    bool noiseTestChannelMessage(const EMANELTE::MHAL::TxControlMessage & txControl,
                                 const EMANELTE::MHAL::ChannelMessage & channel_msg,
                                 SegmentMap & segmentCache,
                                 const uint64_t carrierFrequencyHz,
                                 const uint32_t carrierId);

  private:
    EMANE::NEMId id_;
    EMANE::PlatformServiceProvider * pPlatformService_;
    ENBRadioStatisticManager & statisticManager_;
    std::unique_ptr<DownlinkResourceGridParams> pDownlinkRBParams_;
    std::unique_ptr<UplinkResourceGridParams> pUplinkRBParams_;
    EMANELTE::FrequencyResourceBlockMap rxFreqToRBMap_;
    EMANELTE::FrequencyResourceBlockMap txFreqToRBMap_;
    FrequencySegmentBuilder segmentBuilder_;
    PORManager porManager_;
    float resourceBlockTxPowerdBm_;
    Utils::RandomNumberDistribution<std::mt19937,
                                    std::uniform_real_distribution<float>> RNDZeroToOne_;

    void addTxSegments(const EMANELTE::MHAL::ChannelMessage & channel_msg,
                       const std::uint32_t tti_tx,
                       const std::uint32_t cfi);
  };

}
}
}

#endif //EMANELTE_MHAL_ENBRADIOMESSAGEPROCESSOR_H
