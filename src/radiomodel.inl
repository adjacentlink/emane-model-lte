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


#include "radiomodel.h"
#include "mhalphy.h"
#include "basicstatistichelper.h"
#include "emane/frequencysegment.h"
#include "emane/controls/serializedcontrolmessage.h"
#include "emane/controls/frequencycontrolmessage.h"
#include "emane/controls/frequencyofinterestcontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessageformatter.h"
#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/timestampcontrolmessage.h"
#include "emane/configureexception.h"
#include "emane/utils/parameterconvert.h"



namespace {
  const char * pzModuleName_ = "RadioModel";

  inline EMANE::Microseconds tvToMicroseconds(const timeval & tv)
    {
      return EMANE::Microseconds{tv.tv_sec * 1000000 + tv.tv_usec};
    }

  inline timeval tpToTimeval(const EMANE::TimePoint & tp)
   {
      const EMANE::Microseconds usecs{std::chrono::duration_cast<EMANE::Microseconds>(tp.time_since_epoch())};

      return timeval{usecs.count()/1000000, usecs.count()%1000000};
   }
}


template <class RadioStatManager, class MessageProcessor>
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::RadioModel(EMANE::NEMId id,
                                                         EMANE::PlatformServiceProvider * pPlatformService,
                                                         EMANE::RadioServiceProvider * pRadioService) :
  MACLayerImplementor{id, pPlatformService, pRadioService},
  bRunning_{},
  subframeIntervalMicroseconds_{},
  u16SubId_{},
  pcrCurveURI_{},
  maxPropagationDelay_{},
  u64TxSeqNum_{},
  u32NumResourceBlocks_{},
  u32SymbolsPerSlot_{},
  statisticManager_{id, pPlatformService}
{
  // create message processor per carrier
  for(uint32_t idx = 0; idx < MAX_CARRIERS; ++idx)
   {
     messageProcessor_[idx] = new MessageProcessor{id, pPlatformService, statisticManager_};
   }
}


template <class RadioStatManager, class MessageProcessor>
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::~RadioModel()
{}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::initialize(EMANE::Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s",
                          pzModuleName_,
                          id_,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();


  configRegistrar.registerNonNumeric<std::string>("subid",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"65533"},
                                                  "EMANE Lte Radio Model SubId");

  configRegistrar.registerNonNumeric<std::string>("maxpropagationdelay",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"0"},
                                                  "EMANE Lte Radio Model Max Propagation Delay");

  configRegistrar.registerNonNumeric<std::string>("pcrcurveuri",
                                                  EMANE::ConfigurationProperties::REQUIRED,
                                                  {},
                                                  "Defines the URI of the Packet Completion Rate (PCR) curve "
                                                  "file. The PCR curve file contains probability of reception curves "
                                                  "as a function of Signal to Interference plus Noise Ratio (SINR).");

  configRegistrar.registerNonNumeric<std::string>("resourceblocktxpower",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"0.0"},
                                                  "The transmit power per LTE Resource Block in dBm.");

  statisticManager_.registerStatistics(registrar);

  auto & statisticRegistrar = registrar.statisticRegistrar();

  pSubframeReceiveCounts_ =
    statisticRegistrar.registerTable<std::uint64_t>(
      "SubframeReceiveCounts",
      {"NEMId", "NumPass", "DropPropDelay", "DropFreqMismatch", "DropDirection", "Time"},
      EMANE::StatisticProperties::NONE,
      "Tally of results from checks on subframe receive messages organized by sender: "
      "NumPass - subframe passes all checks. "
      "DropPropDelay - subframe dropped because it exceeds the value fo maxpropagationdelay parameter. "
      "DropFreqMismatch - subframe dropped because the transmitter and receiver are not on the same carrier frequency. "
      "DropDirection - subframe dropped if an uplink message is expected and a downlink message is received, and vice versa. "
      "Time - the last time a check occurred.");
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::configure(const EMANE::ConfigurationUpdate & update)
{
   for(const auto & item : update)
     {
      if(item.first == "subid")
        {
          u16SubId_ = EMANE::Utils::ParameterConvert(item.second[0].asString()).toUINT16();
 
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::INFO_LEVEL,
                                  "%s %03hu %s: %s = %hu",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16SubId_);
        }
      else if(item.first == "maxpropagationdelay")
        {
          maxPropagationDelay_ = EMANE::Microseconds(EMANE::Utils::ParameterConvert(item.second[0].asString()).toUINT64());
 
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::INFO_LEVEL,
                                  "%s %03hu %s: %s = %lu",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxPropagationDelay_.count());
        }
      else if(item.first == "pcrcurveuri")
        {
          pcrCurveURI_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::INFO_LEVEL,
                                  "%s %03hu %s: %s = %s",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  pcrCurveURI_.c_str());

          for(uint32_t idx = 0; idx < MAX_CARRIERS; ++idx)
           {
             messageProcessor_[idx]->loadPCRFile(pcrCurveURI_);
           }
        }
      else if(item.first == "resourceblocktxpower")
        {
          float resourceBlockTxPowerdBm{EMANE::Utils::ParameterConvert(item.second[0].asString()).toFloat()};

          for(uint32_t idx = 0; idx < MAX_CARRIERS; ++idx)
           {
             messageProcessor_[idx]->setTxPower(resourceBlockTxPowerdBm);
           }

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::INFO_LEVEL,
                                  "%s %03hu %s: %s = %0.1f",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  resourceBlockTxPowerdBm);
        }
      else
        {
          throw EMANE::makeException<EMANE::ConfigureException>("EMANE::Models::LTE::RadioModel: "
                                                                "Unexpected configuration item %s",
                                                                item.first.c_str());
        }
    }
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s",
                          pzModuleName_,
                          id_,
                          __func__);

  bRunning_ = true;
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::postStart()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s",
                          pzModuleName_,
                          id_,
                          __func__);
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s",
                          pzModuleName_,
                          id_,
                          __func__);

  bRunning_ = false;
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s",
                          pzModuleName_,
                          id_,
                          __func__);
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processDownstreamControl(const EMANE::ControlMessages &)
{
  // no-op
}
 

        
template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processDownstreamPacket(EMANE::DownstreamPacket &,
                                                                           const EMANE::ControlMessages &)
{
  // no-op
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processUpstreamControl(const EMANE::ControlMessages & /* msgs */)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s:XXX TODO",
                          pzModuleName_,
                          id_,
                          __func__);
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processTimedEvent(EMANE::TimerEventId eventId,
                       const EMANE::TimePoint & /* expireTime */,
                       const EMANE::TimePoint & /* scheduleTime */,
                       const EMANE::TimePoint & /* fireTime */,
                       const void * /* arg */)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s: unexpected event id %ld",
                          pzModuleName_,
                          id_,
                          __func__,
                          eventId);
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setMHAL(EMANELTE::MHAL::PHY::MHALPHY * pMHAL)
{
  pMHAL_ = pMHAL;
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setSubframeInterval(std::uint32_t sfIntervalMsec)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s: sf_interval_msec %u",
                          pzModuleName_,
                          id_,
                          __func__,
                          sfIntervalMsec);

  subframeIntervalMicroseconds_ = EMANE::Microseconds(sfIntervalMsec * 1000);
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setSymbolsPerSlot(std::uint32_t symbolsPerSlot)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s: symbols_per_slot %u",
                          pzModuleName_,
                          id_,
                          __func__,
                          symbolsPerSlot);

  u32SymbolsPerSlot_ = symbolsPerSlot;
}




template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setNumResourceBlocks(std::uint32_t numResourceBlocks)
{
  EMANELTE::FrequencyHz halfChannelBandwidthHz{
    numResourceBlocks * EMANELTE::ResourceBlockBandwidthHz + EMANELTE::HalfResourceBlockBandwidthHz};

  for(const auto & entry : frequencyTable_)
   {
     const auto rxFreqHz = entry.second.first;
     const auto txFreqHz = entry.second.second;

     const auto carrierIndex = entry.first;

     LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                             EMANE::INFO_LEVEL,
                             "%s %03hu %s: carrierIndex=%u, numResourceBlocks=%d, rbBandwidth=%lu, 1/2rbBndwidth=%lu, 1/2chanBandwdith=%lu, rxFreq=%lu Hz, txFreq=%lu Hz",
                             pzModuleName_,
                             id_,
                             __func__,
                             carrierIndex,
                             numResourceBlocks,
                             EMANELTE::ResourceBlockBandwidthHz,
                             EMANELTE::HalfResourceBlockBandwidthHz,
                             halfChannelBandwidthHz,
                             rxFreqHz,
                             txFreqHz);

     // Don't allow configured channel to have negative frequencies
     if(halfChannelBandwidthHz > rxFreqHz || halfChannelBandwidthHz > txFreqHz)
      {
        throw EMANE::makeException<EMANE::ConfigureException>("Invalid configuration, first resource block crosses 0 Hz.");
      }
   }

  u32NumResourceBlocks_ = numResourceBlocks;
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setFrequencies(uint32_t carrierIndex,
                                                                                        EMANELTE::FrequencyHz carrierRxFrequencyHz,
                                                                                        EMANELTE::FrequencyHz carrierTxFrequencyHz,
                                                                                        bool clearCache)
{
  if(clearCache)
   {
     frequencyTable_.clear();
     rxCarrierFrequencyToIndexTable_.clear();
     txCarrierFrequencyToIndexTable_.clear();
     rxCarriersOfInterest_.clear();
   }

  // save rx/tx frequencyHz per carrier id
  frequencyTable_[carrierIndex] = FrequencyPair{carrierRxFrequencyHz, carrierTxFrequencyHz};

  // save carrier id per frequencyHz pair 
  rxCarrierFrequencyToIndexTable_[carrierRxFrequencyHz] = carrierIndex;
  txCarrierFrequencyToIndexTable_[carrierTxFrequencyHz] = carrierIndex;

  rxCarriersOfInterest_.insert(carrierRxFrequencyHz);

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::INFO_LEVEL,
                          "%s %03hu %s: numCarriers=%zu, carrierIndex=%u, rxFreq=%lu MHz, txFreq=%lu MHz",
                          pzModuleName_,
                          id_,
                          __func__,
                          frequencyTable_.size(),
                          carrierIndex,
                          carrierRxFrequencyHz/1000000,
                          carrierTxFrequencyHz/1000000);
}



template <class RadioStatManager, class MessageProcessor>
EMANELTE::FrequencyHz
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getRxResourceBlockFrequency(std::uint32_t resourceBlockIndex, std::uint64_t rx_freq_hz)
{
  return getResourceBlockFrequency(resourceBlockIndex, rx_freq_hz, u32NumResourceBlocks_);
}



template <class RadioStatManager, class MessageProcessor>
EMANELTE::FrequencyHz
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getTxResourceBlockFrequency(std::uint32_t resourceBlockIndex, std::uint64_t tx_freq_hz)
{
  return getResourceBlockFrequency(resourceBlockIndex, tx_freq_hz, u32NumResourceBlocks_);
}



template <class RadioStatManager, class MessageProcessor>
EMANELTE::FrequencyHz
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getResourceBlockFrequency(std::uint64_t resourceBlockIndex,
                                                                                              EMANELTE::FrequencyHz centerFreq,
                                                                                              std::uint64_t numResourceBlocks) const
{
  return centerFreq + ((resourceBlockIndex - numResourceBlocks/2) * 180000) + (((numResourceBlocks + 1) % 2) * 90000);
}



template <class RadioStatManager, class MessageProcessor>
EMANE::SpectrumWindow
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getNoise(EMANELTE::FrequencyHz frequency, 
                                                                             const EMANE::Microseconds & span, 
                                                                             const EMANE::TimePoint & sor)
{
  EMANE::SpectrumWindow spectrumWindow;

  try 
    {
      spectrumWindow = pRadioService_->spectrumService().request(frequency, span, sor);
    } 
  catch(EMANE::SpectrumServiceException & exp) 
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "%s %03hu %s: %s",
                              pzModuleName_,
                              id_,
                              __func__,
                              exp.what());
    }

  return spectrumWindow;
}



template <class RadioStatManager, class MessageProcessor>
double EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getReceiverSensitivitydBm()
{
  return pRadioService_->spectrumService().getReceiverSensitivitydBm();
}



template <class RadioStatManager, class MessageProcessor>
EMANE::FrequencySet EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getFrequencies()
{
  return pRadioService_->spectrumService().getFrequencies();
}



template <class RadioStatManager, class MessageProcessor>
void
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setFrequenciesOfInterest(bool searchMode, bool clearCache)
{
  if(clearCache)
   {
     rxFrequenciesHz_.clear();
     txFrequenciesHz_.clear();
   }

  for(const auto & entry : frequencyTable_)
   {
     const auto rxFreqHz = entry.second.first;
     const auto txFreqHz = entry.second.second;

     const auto carrierIndex = entry.first;

     LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                             EMANE::INFO_LEVEL,
                             "MACI %03hu %s::%s: carrierIndex=%u, searchMode=%d, rxFreq=%lu Hz, txFreq=%lu Hz",
                             id_,
                             pzModuleName_,
                             __func__,
                             carrierIndex,
                             searchMode,
                             rxFreqHz,
                             txFreqHz);

     EMANELTE::FrequencyResourceBlockMap txFreqToRBMap{};
     EMANELTE::FrequencyResourceBlockMap rxFreqToRBMap{};

     for(uint32_t rbidx = 0; rbidx < u32NumResourceBlocks_; ++rbidx)
      {
        const auto frequencyHz = getTxResourceBlockFrequency(rbidx, txFreqHz); // tx frequencyHz

        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                EMANE::DEBUG_LEVEL,
                                "%s %03hu %s: carrierIndex=%u, rbidx=%d txfreq=%lu",
                                pzModuleName_,
                                id_,
                                __func__,
                                carrierIndex,
                                rbidx,
                                frequencyHz);

        txFreqToRBMap.emplace(frequencyHz, rbidx);
        txFrequenciesHz_.emplace(frequencyHz);
      }

     for(uint32_t rbidx = 0; rbidx < u32NumResourceBlocks_; ++rbidx)
      {
        const auto frequencyHz = getRxResourceBlockFrequency(rbidx, rxFreqHz); // rx frequencyHz

        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                EMANE::DEBUG_LEVEL,
                                "%s %03hu %s: carrierIndex=%u, rbidx=%d rxfreq=%lu",
                                pzModuleName_,
                                id_,
                                __func__,
                                carrierIndex,
                                rbidx,
                                frequencyHz);

        rxFreqToRBMap.emplace(frequencyHz, rbidx);
        rxFrequenciesHz_.emplace(frequencyHz);
      }

    // During cell searchMode the ue doesn't know the enb bandwidth or the frequencies
    // which the enb is transmitting on. The enb may be configured for a bandwidth
    // that contains an even or an odd number of resource blocks. When told to
    // search, register FOI for the largest even (100) and odd (75) resource block
    // bandwidths to ensure receipt of enb packets from the phy for any enb.
    if(searchMode)
      {
        if(u32NumResourceBlocks_ != 100)
          {
            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    EMANE::ERROR_LEVEL,
                                    "%s %03hu %s: Attempt to do cell search with numResourceBlocks set to %d. Ignoring.",
                                    pzModuleName_,
                                   id_,
                                   __func__,
                                   u32NumResourceBlocks_);
           return;
         }

        EMANELTE::FrequencyResourceBlockMap rx75FreqToRBMap{};

        for(uint32_t rbidx = 0; rbidx < 75; ++rbidx)
          {
            const auto frequencyHz = getResourceBlockFrequency(rbidx, rxFreqHz, 75); // rx frequencyHz
 
            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    EMANE::DEBUG_LEVEL,
                                    "%s %03hu %s: rbidx=%d rxfreq=%lu",
                                    pzModuleName_,
                                    id_,
                                    __func__,
                                    rbidx,
                                    frequencyHz);

            rx75FreqToRBMap.emplace(frequencyHz, rbidx);
            rxFrequenciesHz_.emplace(frequencyHz);
          }

      messageProcessor_[carrierIndex]->swapSearchFrequencyMaps(rxFreqToRBMap, rx75FreqToRBMap, txFreqToRBMap);
    }
  else
    {
      messageProcessor_[carrierIndex]->swapFrequencyMaps(rxFreqToRBMap, txFreqToRBMap);
    }
  }

#if 0 
  size_t idx = 0; 
  for(const auto & frequencyHz : rxFrequenciesHz_)
   {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::INFO_LEVEL,
                              "%s %03hu %s: idx %zu, rxfreq=%lu",
                              pzModuleName_,
                              id_,
                              __func__,
                              idx++,
                              frequencyHz);
   }

  idx = 0; 
  for(const auto & frequencyHz : txFrequenciesHz_)
   {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::INFO_LEVEL,
                              "%s %03hu %s: idx %zu, txfreq=%lu",
                              pzModuleName_,
                              id_,
                              __func__,
                              idx++,
                              frequencyHz);
   }
#endif

  statisticManager_.updateFrequencies(rxFrequenciesHz_, txFrequenciesHz_);

  sendDownstreamControl({EMANE::Controls::FrequencyOfInterestControlMessage::create(EMANELTE::ResourceBlockBandwidthHz,
                                                                                    rxFrequenciesHz_)});
}



template <class RadioStatManager, class MessageProcessor>
void
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::sendDownstreamMessage(const EMANELTE::MHAL::Data & data,
                                                                                          EMANELTE::MHAL::TxControlMessage & txControl,
                                                                                          const EMANE::TimePoint & timestamp)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          EMANE::DEBUG_LEVEL,
                          "%s %03hu %s data_len %3zu, tti_tx %05u, sf_time %d:%06d, timestamp %f",
                          pzModuleName_,
                          id_,
                          __func__,
                          data.length(),
                          txControl.tti_tx(),
                          txControl.sf_time().ts_sec(),
                          txControl.sf_time().ts_usec(),
                          timestamp.time_since_epoch().count()/1e9);

   // MUX all frequency segments for the msg
   EMANE::FrequencySegments frequencySegments;

   // txControl carrier map uses center frequency as the key
   for(const auto & carrier : txControl.carriers())
    {
      const auto carrierFrequencyHz = carrier.first;

      const auto iter = txCarrierFrequencyToIndexTable_.find(carrierFrequencyHz);

      EMANELTE::FrequencySet frequenciesThisCarrier;

      // sanity check
      if(iter != txCarrierFrequencyToIndexTable_.end())
       {
         const auto carrierIndex = iter->second;

         // get the all frequency segments for this carrier
         const auto segments = messageProcessor_[carrierIndex]->buildFrequencySegments(txControl, carrierFrequencyHz);

         for(auto segment : segments)
          {
            const auto frequencyHz = segment.getFrequencyHz();

            // unique list of subchannel frequencies
            if(frequenciesThisCarrier.insert(frequencyHz).second)
             {
               (*txControl.mutable_carriers())[carrierFrequencyHz].add_sub_channels(frequencyHz);
             }

            frequencySegments.push_back(segment);
          }
       }
      else
       {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                 EMANE::INFO_LEVEL,
                                 "MACI %03hu %s::%s: skip carrier %lu Hz, no matching carriers",
                                 id_,
                                 pzModuleName_,
                                 __func__,
                                 carrierFrequencyHz);
       }
    }

#if 0
   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           EMANE::INFO_LEVEL,
                           "%s %03hu %s %s",
                           pzModuleName_,
                           id_,
                           __func__,
                           txControl.DebugString().c_str());
#endif

   const EMANE::Microseconds sfDuration{txControl.subframe_duration_microsecs()};

   const EMANE::ControlMessages msgs = 
      {EMANE::Controls::FrequencyControlMessage::create(EMANELTE::ResourceBlockBandwidthHz, frequencySegments),
       EMANE::Controls::TimeStampControlMessage::create(timestamp)};     // sot

   std::string sSerialization{};

   txControl.SerializeToString(&sSerialization);

   EMANE::DownstreamPacket pkt{EMANE::PacketInfo{id_,
                                                 EMANE::NEM_BROADCAST_MAC_ADDRESS,
                                                 0,
                                                 EMANE::Clock::now()},  // clock tx time (creation time)
                               data.data(), data.length()};

   pkt.prepend(sSerialization.c_str(), sSerialization.length());

   pkt.prependLengthPrefixFraming(sSerialization.length());

   sendDownstreamPacket(EMANE::CommonMACHeader{u16SubId_, ++u64TxSeqNum_}, pkt, msgs);
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processUpstreamPacket(const EMANE::CommonMACHeader & commonMACHeader,
                                                                         EMANE::UpstreamPacket & pkt,
                                                                         const EMANE::ControlMessages & controlMessages)
{
  const EMANE::TimePoint tpRxTime = EMANE::Clock::now();

  if(commonMACHeader.getRegistrationId() != u16SubId_)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "MACI %03hu %s::%s: MAC Registration Id %hu does not match our Id %hu, drop.",
                              id_,
                              pzModuleName_,
                              __func__,
                              commonMACHeader.getRegistrationId(),
                              u16SubId_);

      return;
    }

  const EMANE::PacketInfo & pktInfo{pkt.getPacketInfo()};

  const EMANE::Controls::ReceivePropertiesControlMessage * pReceivePropertiesControlMessage = NULL;

  const EMANE::Controls::FrequencyControlMessage * pFrequencyControlMessage = NULL;

  for(auto iter = controlMessages.begin(); iter != controlMessages.end(); ++iter)
    {
      switch((*iter)->getId())
        {
        case EMANE::Controls::ReceivePropertiesControlMessage::IDENTIFIER:
          {
            pReceivePropertiesControlMessage =
              static_cast<const EMANE::Controls::ReceivePropertiesControlMessage *>(*iter);
          }
          break;

        case EMANE::Controls::FrequencyControlMessage::IDENTIFIER:
          {
            pFrequencyControlMessage =
              static_cast<const EMANE::Controls::FrequencyControlMessage *>(*iter);
          }
          break;
        }
    }

  if(!pReceivePropertiesControlMessage || !pFrequencyControlMessage)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "MACI %03hu %s::%s: phy control "
                              "message not provided from src %hu, drop",
                              id_,
                              pzModuleName_,
                              __func__,
                              pktInfo.getSource());

      return;
    }

  const uint16_t prefixLength{pkt.stripLengthPrefixFraming()};

  if(prefixLength && (pkt.length() >= prefixLength))
    {
      const std::string sSerialization{reinterpret_cast<const char*>(pkt.get()), prefixLength};

      pkt.strip(prefixLength);

      EMANELTE::MHAL::TxControlMessage txControl;

      if(txControl.ParseFromString(sSerialization))
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 EMANE::DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: src %hu, seqnum %lu, span %lu, prop_delay %lu, max_delay %lu",
                                 id_,
                                 pzModuleName_,
                                 __func__,
                                 pktInfo.getSource(),
                                 txControl.tx_seqnum(),
                                 pReceivePropertiesControlMessage->getSpan().count(),
                                 pReceivePropertiesControlMessage->getPropagationDelay().count(),
                                 maxPropagationDelay_.count());

         // check for up/down link type
         if(txControl.message_type() != messageProcessor_[0]->receiveMessageType_)
          {
            updateSubframeDropDirection_i(pktInfo.getSource());

            return;
          }

          // if enabled (!=0) check propagation delay limit
          const bool bPassPropagationDelay = maxPropagationDelay_.count() > 0 ?
                     pReceivePropertiesControlMessage->getPropagationDelay() <= maxPropagationDelay_ : true;

          if(!bPassPropagationDelay)
            {
              updateSubframeDropPropagationDelay_i(pktInfo.getSource());

              return;
            }

          bool bFoundCarrier = false;

          // must have at least 1 matching carrier to pass this message
          for(auto carrier : txControl.carriers())
            {
              // check the carrier frequencyHz, see if it matches one of our carrier rx freqs
              if(rxCarriersOfInterest_.count(carrier.first))
               {
                 bFoundCarrier = true;

                 break;
               }
            }

          if(! bFoundCarrier)
           {
             LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                     EMANE::INFO_LEVEL,
                                     "MACI %03hu %s::%s: drop msg from %hu, no matching carriers",
                                     id_,
                                     pzModuleName_,
                                     __func__,
                                     pktInfo.getSource());

              updateSubframeDropFrequencyMismatch_i(pktInfo.getSource());

              return;
           }

#if 0
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           EMANE::INFO_LEVEL,
                           "%s %03hu %s\n%s",
                           pzModuleName_,
                           id_,
                           __func__,
                           txControl.DebugString().c_str());
#endif

          statisticManager_.updateRxTableCounts(txControl, rxCarriersOfInterest_);

          pMHAL_->handle_upstream_msg(
            EMANELTE::MHAL::Data{reinterpret_cast<const char *>(pkt.get()), pkt.length()},   // opaque data

            EMANELTE::MHAL::RxData{pktInfo.getSource(),                                      // src nem
                txControl.tx_seqnum(),                                                       // tx seqnum
                tpToTimeval(tpRxTime),                                                       // clock rx time
                tpToTimeval(pktInfo.getCreationTime()),                                      // clock tx time
                timeval{txControl.sf_time().ts_sec(),                                        // sf time
                        txControl.sf_time().ts_usec()},
                0.0,                                                                         // peak sum
                0},                                                                          // num samples

            EMANELTE::MHAL::PHY::OTAInfo{pReceivePropertiesControlMessage->getTxTime(),      // emulation sot
                pReceivePropertiesControlMessage->getPropagationDelay(),                     // propagation delay
                pReceivePropertiesControlMessage->getSpan(),                                 // span
                pFrequencyControlMessage->getFrequencySegments()},                           // frequency segments
                txControl);                                                                  // txControl

          updateSubframePass_i(pktInfo.getSource());
        }
      else
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::ERROR_LEVEL,
                                  "%s %03hu %s: serialization error prefix_len %hu, pkt_len %zu",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  prefixLength,
                                  pkt.length());
        }
    }
  else
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "%s %03hu %s: pkt length error prefix_len %hu, pkt_len %zu",
                              pzModuleName_,
                              id_,
                              __func__,
                              prefixLength,
                              pkt.length());
    }
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::updateSubframePass_i(EMANE::NEMId id)
{
  auto iter = subframeReceiveCountDB_.find(id);

  if(iter == subframeReceiveCountDB_.end())
   {
     subframeReceiveCountDB_.insert(std::make_pair(id, SubframeReceiveCountEntry{1,0,0,0})).first;

     EMANELTE::addRowWithCheck_(pSubframeReceiveCounts_,
                                id,
                                {EMANE::Any{std::uint64_t{id}},
                                 EMANE::Any{std::uint64_t{1}},
                                 EMANE::Any{std::uint64_t{0}},
                                 EMANE::Any{std::uint64_t{0}},
                                 EMANE::Any{std::uint64_t{0}},
                                 EMANE::Any{time(NULL)}});
   }
  else
   {
     EMANELTE::setRowWithCheck_(pSubframeReceiveCounts_,
                                id,
                                {EMANE::Any{std::uint64_t{id}},
                                 EMANE::Any{std::uint64_t{++std::get<0>(iter->second)}},
                                 EMANE::Any{std::uint64_t{std::get<1>(iter->second)}},
                                 EMANE::Any{std::uint64_t{std::get<2>(iter->second)}},
                                 EMANE::Any{std::uint64_t{std::get<3>(iter->second)}},
                                 EMANE::Any{time(NULL)}});
   }
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::updateSubframeDropPropagationDelay_i(EMANE::NEMId id)
{
  auto iter = subframeReceiveCountDB_.find(id);

  if(iter == subframeReceiveCountDB_.end())
   {
     subframeReceiveCountDB_.insert(std::make_pair(id, SubframeReceiveCountEntry{0,1,0,0})).first;

     EMANELTE::addRowWithCheck_(pSubframeReceiveCounts_,
                      id,
                      {EMANE::Any{std::uint64_t{id}},
                       EMANE::Any{std::uint64_t{0}},
                       EMANE::Any{std::uint64_t{1}},
                       EMANE::Any{std::uint64_t{0}},
                       EMANE::Any{std::uint64_t{0}},
                       EMANE::Any{time(NULL)}});
   }
  else
   {
     EMANELTE::setRowWithCheck_(pSubframeReceiveCounts_,
                                id,
                                {EMANE::Any{std::uint64_t{id}},
                                 EMANE::Any{std::uint64_t{std::get<0>(iter->second)}},
                                 EMANE::Any{std::uint64_t{++std::get<1>(iter->second)}},
                                 EMANE::Any{std::uint64_t{std::get<2>(iter->second)}},
                                 EMANE::Any{std::uint64_t{std::get<3>(iter->second)}},
                                 EMANE::Any{time(NULL)}});
   }
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::updateSubframeDropFrequencyMismatch_i(EMANE::NEMId id)
{
  auto iter = subframeReceiveCountDB_.find(id);

  if(iter == subframeReceiveCountDB_.end())
    {
      subframeReceiveCountDB_.insert(std::make_pair(id, SubframeReceiveCountEntry{0,0,1,0})).first;

      EMANELTE::addRowWithCheck_(pSubframeReceiveCounts_,
                                 id,
                                 {EMANE::Any{std::uint64_t{id}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{std::uint64_t{1}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{time(NULL)}});
    }
  else
    {
      EMANELTE::setRowWithCheck_(pSubframeReceiveCounts_,
                                 id,
                                 {EMANE::Any{std::uint64_t{id}},
                                  EMANE::Any{std::uint64_t{std::get<0>(iter->second)}},
                                  EMANE::Any{std::uint64_t{std::get<1>(iter->second)}},
                                  EMANE::Any{std::uint64_t{++std::get<2>(iter->second)}},
                                  EMANE::Any{std::uint64_t{std::get<3>(iter->second)}},
                                  EMANE::Any{time(NULL)}});
    }
}


template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::updateSubframeDropDirection_i(EMANE::NEMId id)
{
  auto iter = subframeReceiveCountDB_.find(id);

  if(iter == subframeReceiveCountDB_.end())
    {
      subframeReceiveCountDB_.insert(std::make_pair(id, SubframeReceiveCountEntry{0,0,0,1})).first;

      EMANELTE::addRowWithCheck_(pSubframeReceiveCounts_,
                                 id,
                                 {EMANE::Any{std::uint64_t{id}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{std::uint64_t{0}},
                                  EMANE::Any{std::uint64_t{1}},
                                  EMANE::Any{time(NULL)}});
    }
  else
    {
      EMANELTE::setRowWithCheck_(pSubframeReceiveCounts_,
                                 id,
                                 {EMANE::Any{std::uint64_t{id}},
                                  EMANE::Any{std::uint64_t{std::get<0>(iter->second)}},
                                  EMANE::Any{std::uint64_t{std::get<1>(iter->second)}},
                                  EMANE::Any{std::uint64_t{std::get<2>(iter->second)}},
                                  EMANE::Any{std::uint64_t{++std::get<3>(iter->second)}},
                                  EMANE::Any{time(NULL)}});
    }
}


template <class RadioStatManager, class MessageProcessor>
bool EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::noiseTestChannelMessage(
   const EMANELTE::MHAL::TxControlMessage & txControl,
   const EMANELTE::MHAL::ChannelMessage & channel_msg,
   SegmentMap & segmentCache,
   std::uint64_t carrierFrequencyHz)
{
  // check rx carrier table for tx frequencyHz
  const auto iter = rxCarrierFrequencyToIndexTable_.find(carrierFrequencyHz);

  if(iter != rxCarrierFrequencyToIndexTable_.end())
   {
     return messageProcessor_[iter->second]->noiseTestChannelMessage(txControl, channel_msg, segmentCache, carrierFrequencyHz);
   }
  else
   {
     return false;
   }
}


template <class RadioStatManager, class MessageProcessor>
EMANELTE::FrequencySet EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getCarriersOfInterest() const
{
  return rxCarriersOfInterest_;
}



namespace EMANE
{
namespace Models
{
namespace LTE
{

template class RadioModel<UERadioStatisticManager, UEMessageProcessor>;
typedef RadioModel<UERadioStatisticManager, UEMessageProcessor> UERadioModel;

template class RadioModel<ENBRadioStatisticManager, ENBMessageProcessor>;
typedef RadioModel<ENBRadioStatisticManager, ENBMessageProcessor> ENBRadioModel;

}
}
}
