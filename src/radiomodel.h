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


#ifndef EMANELTE_MHAL_RADIOMODEL_H
#define EMANELTE_MHAL_RADIOMODEL_H

#include "ltedefs.h"
#include "mhalphy.h"
#include "segmentmap.h"

#include "libemanelte/mhal.h"

#include "emane/maclayerimpl.h"
#include "emane/types.h"
#include "emane/application/logger.h"
#include "emane/frequencysegment.h"
#include "emane/platformserviceprovider.h"
#include "emane/utils/netutils.h"
#include "emane/utils/spectrumwindowutils.h"

#include "enbmessageprocessor.h"
#include "enbradiostatisticmanager.h"
#include "uemessageprocessor.h"
#include "ueradiostatisticmanager.h"


namespace EMANE {
namespace Models {
namespace LTE {

// relative rx/tx per node type ue/enb
using FrequencyPair = std::pair<std::uint64_t, std::uint64_t>;

// frequency, spectrum window
using SpectrumWindowCache = std::map<std::uint64_t, EMANE::SpectrumWindow>;

using FrequencySegmentParams = std::pair<EMANELTE::FrequencyHz, EMANELTE::BandwidthHz>;

using ResourceBlockMap = std::map<uint32_t, FrequencySegmentParams>;

const uint32_t MAX_CARRIERS = 5;

template <class RadioStatManager, class MessageProcessor>
class RadioModel : public EMANE::MACLayerImplementor
  {
    public:
      RadioModel(EMANE::NEMId id,
                 EMANE::PlatformServiceProvider *pPlatformService,
                 EMANE::RadioServiceProvider * pRadioService);

      ~RadioModel() override;

      void initialize(EMANE::Registrar & registrar) override;

      void configure(const EMANE::ConfigurationUpdate & update) override;

      void start() override;

      void postStart() override;

      void stop() override;

      void destroy() throw() override;

      void processDownstreamControl(const EMANE::ControlMessages & msgs) override;
 
        
      void processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                                   const EMANE::ControlMessages & msgs) override;


      void processUpstreamPacket(const EMANE::CommonMACHeader & hdr,
                                 EMANE::UpstreamPacket & pkt,
                                 const EMANE::ControlMessages & msgs) override;
  
      void processUpstreamControl(const EMANE::ControlMessages & msgs) override;
    
      void processTimedEvent(EMANE::TimerEventId eventId,
                             const EMANE::TimePoint & expireTime,
                             const EMANE::TimePoint & scheduleTime,
                             const EMANE::TimePoint & fireTime,
                             const void * arg) override;

      void sendDownstreamMessage(const EMANELTE::MHAL::Data & data,
                                 EMANELTE::MHAL::TxControlMessage & control,
                                 const EMANE::TimePoint & timestamp);

      void setMHAL(EMANELTE::MHAL::PHY::MHALPHY * pMHAL);

      RadioStatManager & getStatisticManager() { return statisticManager_; }

      void setSubframeInterval(std::uint32_t sfIntervalMsec);

      void setSymbolsPerSlot(std::uint32_t symbolsPerSlot);

      void setFrequencies(uint32_t carrierId,
                          EMANELTE::FrequencyHz rxFrequency,
                          EMANELTE::FrequencyHz txFrequency);

      void setNumResourceBlocks(std::uint32_t numResourceBlocks, std::uint32_t carriedId, bool search=false);

      EMANELTE::FrequencyHz getRxResourceBlockFrequency(std::uint32_t resourceBlockIndex, std::uint64_t rx_freq_hz);

      EMANELTE::FrequencyHz getTxResourceBlockFrequency(std::uint32_t resourceBlockIndex, std::uint64_t tx_freq_hz);

      EMANE::SpectrumWindow getNoise(EMANELTE::FrequencyHz frequency, 
                                     const EMANE::Microseconds & span, 
                                     const EMANE::TimePoint & sor);

      double getReceiverSensitivitydBm();

      EMANE::FrequencySet getFrequencies();

      bool noiseTestChannelMessage(const EMANELTE::MHAL::TxControlMessage & txControl,
                                            const EMANELTE::MHAL::ChannelMessage & channel_msg,
                                            SegmentMap & segmentCache);

    private:
      bool bRunning_;
      EMANE::Microseconds subframeIntervalMicroseconds_;

      std::uint16_t u16SubId_;
      std::string pcrCurveURI_;
      EMANE::Microseconds maxPropagationDelay_;

      using FrequencyTable = std::map<std::uint32_t, FrequencyPair>;

      using CarrierTable   = std::map<FrequencyPair, std::uint32_t>;

      // track frequencies by carrier
      FrequencyTable frequencyTable_;

      // track carrier by freq pair
      CarrierTable   carrierTable_;

      std::uint64_t u64TxSeqNum_;
      std::set<std::uint64_t> rxFrequencySetHz_;
      std::uint32_t u32NumResourceBlocks_;
      std::uint16_t u32SymbolsPerSlot_;

      RadioStatManager statisticManager_;
      MessageProcessor *messageProcessor_[MAX_CARRIERS]; // XXX_CC just handle each carrier here ???

      EMANELTE::MHAL::PHY::MHALPHY * pMHAL_;

      EMANE::StatisticTable<std::uint64_t> * pSubframeReceiveCounts_ = NULL;

      // NumPass, DropPropDelay, DropFreqMismatch, DropDirection
      using SubframeReceiveCountEntry = std::tuple<size_t, size_t, size_t, size_t>;
      // NEMId key
      using SubframeReceiveCountDB = std::map<EMANE::NEMId, SubframeReceiveCountEntry>;
      SubframeReceiveCountDB subframeReceiveCountDB_;

      void setFrequenciesOfInterest(bool search, std::uint32_t carrierId);

      EMANELTE::FrequencyHz getResourceBlockFrequency(std::uint64_t resourceBlockIndex,
                                                      EMANELTE::FrequencyHz centerFreq,
                                                      std::uint64_t numResourceBlocks) const;

      void updateSubframePass_i(EMANE::NEMId id);

      void updateSubframeDropPropagationDelay_i(EMANE::NEMId id);

      void updateSubframeDropFrequencyMismatch_i(EMANE::NEMId id);

      void updateSubframeDropDirection_i(EMANE::NEMId id);
   };
}
}
}

#include "radiomodel.inl"

#endif
