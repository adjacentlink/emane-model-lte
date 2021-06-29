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
#include "emane/controls/timestampcontrolmessage.h"
#include "emane/controls/mimotransmitpropertiescontrolmessage.h"
#include <emane/controls/mimoreceivepropertiescontrolmessage.h>
#include <emane/controls/mimoreceivepropertiescontrolmessageformatter.h>
#include <emane/controls/rxantennaaddcontrolmessage.h>
#include <emane/controls/rxantennaremovecontrolmessage.h>
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
  bAntennaInit_{},
  statisticManager_{id, pPlatformService}
{
  // create message processor per carrier
  for(uint32_t idx = 0; idx < EMANELTE::MAX_CARRIERS; ++idx)
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

  configRegistrar.registerNonNumeric<std::string>("antenna",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"omni"},
                                                  "Defines the antenna properties 'omni' or sector{index,az,el},sector{index,az,el},...");


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

          for(uint32_t idx = 0; idx < EMANELTE::MAX_CARRIERS; ++idx)
           {
             messageProcessor_[idx]->loadPCRFile(pcrCurveURI_);
           }
        }
      else if(item.first == "resourceblocktxpower")
        {
          float resourceBlockTxPowerdBm{EMANE::Utils::ParameterConvert(item.second[0].asString()).toFloat()};

          for(uint32_t idx = 0; idx < EMANELTE::MAX_CARRIERS; ++idx)
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
      else if(item.first == "antenna")
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::INFO_LEVEL,
                                  "%s %03hu %s: %s = %s",
                                  pzModuleName_,
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  item.second[0].asString().c_str());

          sAntennaInfo_ = item.second[0].asString();
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

   txAntennas_.clear();

   antennaPointings_.clear();

   if(sAntennaInfo_ == "omni")
    {
      auto txAntenna = Antenna::createIdealOmni(0, 0.0); // antenna index 0, gain 0

      txAntenna.setFrequencyGroupIndex(0);

      txAntenna.setBandwidthHz(EMANELTE::ResourceBlockBandwidthHz);

      txAntennas_.emplace_back(txAntenna);
    }
   else
    {
      const std::string tag = "sector";

      size_t pos = 0;

      while((pos = sAntennaInfo_.find(tag, pos)) != std::string::npos)
       {
          uint8_t profile = 0;
          float az = 0, el = 0;

          const int numArgs = sscanf(sAntennaInfo_.data() + pos + tag.length(), "{%hhu,%f,%f}", &profile, &az, &el);

          if(numArgs != 3)
           {
             throw EMANE::makeException<EMANE::ConfigureException>("EMANE::Models::LTE::RadioModel: Invalid antenna info %s", sAntennaInfo_.c_str());
           }

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                   EMANE::INFO_LEVEL,
                                   "%s %03hu %s antenna index %zu, profile %u, az %f, el %f",
                                   pzModuleName_,
                                   id_,
                                   __func__,
                                   txAntennas_.size(),
                                   profile,
                                   az, el);

          const auto pointing = Antenna::Pointing(profile, az, el); // profile, az, el

          auto txAntenna = Antenna::createProfileDefined(txAntennas_.size(), pointing); // antenna index, pointing

          txAntenna.setFrequencyGroupIndex(txAntennas_.size());

          txAntenna.setBandwidthHz(EMANELTE::ResourceBlockBandwidthHz);

          antennaPointings_.emplace_back(pointing);

          txAntennas_.emplace_back(txAntenna);

          pos += tag.length();
       }
   }

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
                          "%s %03hu %s: numCarriers=%zu, carrierIndex=%u, rxFreq=%lu Hz, txFreq=%lu Hz",
                          pzModuleName_,
                          id_,
                          __func__,
                          frequencyTable_.size(),
                          carrierIndex,
                          carrierRxFrequencyHz,
                          carrierTxFrequencyHz);
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
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getNoise(const uint32_t antennaIndex,
                                                                             const EMANELTE::FrequencyHz frequency, 
                                                                             const EMANE::Microseconds & span, 
                                                                             const EMANE::TimePoint & sor)
{
  const auto tp = EMANE::Clock::now();

  EMANE::SpectrumWindow spectrumWindow;

  try 
    {
      spectrumWindow = pRadioService_->spectrumService().requestAntenna(antennaIndex, frequency, span, sor);
    } 
  catch(EMANE::SpectrumServiceException & exp) 
    {
      const auto dt = std::chrono::duration_cast<EMANE::Microseconds>(tp - sor);

      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "%s %03hu %s: frequency %lu, now %lf, sor %lf, span %ld, dt %ld usec, reason %s",
                              pzModuleName_,
                              id_,
                              __func__,
                              frequency,
                              tp.time_since_epoch().count()  / 1e9,
                              sor.time_since_epoch().count() / 1e9,
                              span.count(),
                              dt.count(),
                              exp.what());
    }

  return spectrumWindow;
}



template <class RadioStatManager, class MessageProcessor>
float EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getReceiverSensitivitydBm()
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
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::setFrequenciesOfInterest(bool searchMode)
{
  // all freqs of interest
  FrequencySet allRxFrequenciesHz, allTxFrequenciesHz;

  // rx frequencies per carrier
  std::map<uint32_t, FrequencySet> rxFrequencyTableHz; 

  // for each carrier
  for(const auto & entry : frequencyTable_)
   {
     const auto & rxFreqHz = entry.second.first;
     const auto & txFreqHz = entry.second.second;

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

        allTxFrequenciesHz.emplace(frequencyHz);
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

        allRxFrequenciesHz.emplace(frequencyHz);

        rxFrequencyTableHz[carrierIndex].insert(frequencyHz);
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

           allRxFrequenciesHz.emplace(frequencyHz);

           rxFrequencyTableHz[carrierIndex].insert(frequencyHz);
         }

       messageProcessor_[carrierIndex]->swapSearchFrequencyMaps(rxFreqToRBMap, rx75FreqToRBMap, txFreqToRBMap);
     }
    else
     {
       messageProcessor_[carrierIndex]->swapFrequencyMaps(rxFreqToRBMap, txFreqToRBMap);
     }
  }


  ControlMessages controlMsgs; 

  // remove old antenna(s)
  if(bAntennaInit_)
   {
     if(sAntennaInfo_ == "omni")
      {
         controlMsgs.emplace_back(Controls::RxAntennaRemoveControlMessage::create(0));
      }
     else
      {
        for(size_t antennaIndex = 0; antennaIndex < antennaPointings_.size(); ++antennaIndex)
         {
           controlMsgs.emplace_back(Controls::RxAntennaRemoveControlMessage::create(antennaIndex));
         }
      }
   }
  else
   {
     bAntennaInit_ = true;
   }


  // add new antenna freqs
  if(sAntennaInfo_ == "omni")
   {
      const auto rxAntenna = Antenna::createIdealOmni(0, 0.0); // antenna index 0, gain 0

      controlMsgs.emplace_back(Controls::RxAntennaAddControlMessage::create(rxAntenna, allRxFrequenciesHz));
   }
  else
   {
     for(size_t antennaIndex = 0; antennaIndex < antennaPointings_.size(); ++antennaIndex)
      {
        // create profile defined
        const auto rxAntenna = Antenna::createProfileDefined(antennaIndex, antennaPointings_[antennaIndex]);

        // add antenna
        controlMsgs.emplace_back(Controls::RxAntennaAddControlMessage::create(rxAntenna, rxFrequencyTableHz[antennaIndex]));
      }
   }

  // update phy
  sendDownstreamControl(controlMsgs);

  // update stats
  statisticManager_.updateFrequencies(allRxFrequenciesHz, allTxFrequenciesHz);
}



template <class RadioStatManager, class MessageProcessor>
void
EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::sendDownstreamMessage(const EMANELTE::MHAL::Data & data,
                                                                                          EMANELTE::MHAL::TxControlMessage & txControl,
                                                                                          const EMANE::TimePoint & tpTxTime)
{
   // MUX all frequency segments for the msg
   FrequencyGroups frequencyGroups;

   for(int idx = 0; idx < txControl.carriers().size(); ++idx)
    {
      auto control = txControl.mutable_carriers(idx);

      const auto carrierFrequencyHz = control->frequency_hz();

      const auto iter = txCarrierFrequencyToIndexTable_.find(carrierFrequencyHz);

      // sanity check
      if(iter != txCarrierFrequencyToIndexTable_.end())
       {
         const auto carrierIndex = iter->second;

         // get the all frequency segments for this carrier
         const auto segments = messageProcessor_[carrierIndex]->buildFrequencySegments(txControl, carrierFrequencyHz);

         EMANELTE::FrequencySet frequencySet;

         for(const auto & segment : segments)
          {
            const auto & segmentFrequencyHz = segment.getFrequencyHz();

            // unique list of subchannel frequencies
            if(frequencySet.insert(segmentFrequencyHz).second)
             {
               control->add_sub_channels(segmentFrequencyHz);
             }
          }

         // load the frequency segments for this carrier
         frequencyGroups.emplace_back(segments);
       }
      else
       {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                 EMANE::ERROR_LEVEL,
                                 "MACI %03hu %s::%s: ignore carrier %lu Hz, no matching carriers",
                                 id_,
                                 pzModuleName_,
                                 __func__,
                                 carrierFrequencyHz);
       }
    }


   std::string sSerialization{};

   txControl.SerializeToString(&sSerialization);

   EMANE::DownstreamPacket pkt{EMANE::PacketInfo{id_,
                                                 EMANE::NEM_BROADCAST_MAC_ADDRESS,
                                                 0,
                                                 EMANE::Clock::now()},  // clock tx time (creation time)
                               data.data(), data.length()};

   pkt.prepend(sSerialization.c_str(), sSerialization.length());

   pkt.prependLengthPrefixFraming(sSerialization.length());
   
   sendDownstreamPacket(EMANE::CommonMACHeader{u16SubId_, ++u64TxSeqNum_},
                        pkt,
                        {Controls::MIMOTransmitPropertiesControlMessage::create(std::move(frequencyGroups), txAntennas_),
                         Controls::TimeStampControlMessage::create(tpTxTime)});
}



template <class RadioStatManager, class MessageProcessor>
void EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::processUpstreamPacket(const EMANE::CommonMACHeader & commonMACHeader,
                                                                         EMANE::UpstreamPacket & pkt,
                                                                         const EMANE::ControlMessages & controlMessages)
{
  const auto tpNow = EMANE::Clock::now();

  const EMANE::PacketInfo & pktInfo{pkt.getPacketInfo()};

  if(commonMACHeader.getRegistrationId() != u16SubId_)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "MACI %03hu %s::%s: MAC Registration Id %hu from src %hu, does not match our Id %hu, drop.",
                              id_,
                              pzModuleName_,
                              __func__,
                              commonMACHeader.getRegistrationId(),
                              pktInfo.getSource(),
                              u16SubId_);

      return;
    }


  const Controls::MIMOReceivePropertiesControlMessage * pMIMOReceivePropertiesControlMessage{};

  for(auto & pControlMessage : controlMessages)
   {
     switch(pControlMessage->getId())
      {
       case Controls::MIMOReceivePropertiesControlMessage::IDENTIFIER:
         {
           pMIMOReceivePropertiesControlMessage =
             static_cast<const Controls::MIMOReceivePropertiesControlMessage *>(pControlMessage);
         } break;
       }
    }

  if(!pMIMOReceivePropertiesControlMessage)
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

     // unpack the tx control info
     if(txControl.ParseFromString(sSerialization))
       {
         const auto propagationDelay = pMIMOReceivePropertiesControlMessage->getPropagationDelay();

         const auto tpTxTime = pMIMOReceivePropertiesControlMessage->getTxTime();

         const auto recvInfos = pMIMOReceivePropertiesControlMessage->getAntennaReceiveInfos();
  
#if 0
         const auto dt = std::chrono::duration_cast<EMANE::Microseconds>(tpTxTime - tpNow);

         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                 EMANE::INFO_LEVEL,
                                 "MACI %03hu %s::%s: src %hu, seqnum %lu, prop_delay %lu, curr_time %f, txTime %f, dt %ld usec",
                                 id_,
                                 pzModuleName_,
                                 __func__,
                                 pktInfo.getSource(),
                                 txControl.tx_seqnum(),
                                 propagationDelay.count(),
                                 tpNow.time_since_epoch().count()/1e9,
                                 tpTxTime.time_since_epoch().count()/1e9,
                                 dt.count());
#endif

        // check for up/down link type
        if(txControl.message_type() != messageProcessor_[0]->receiveMessageType_)
         {
           updateSubframeDropDirection_i(pktInfo.getSource());

           return;
         }

         // if enabled (!=0) check propagation delay limit
         const bool bPassPropagationDelay = maxPropagationDelay_.count() > 0 ? propagationDelay <= maxPropagationDelay_ : true;

         if(!bPassPropagationDelay)
          {
            updateSubframeDropPropagationDelay_i(pktInfo.getSource());

            return;
          }

         bool bFoundCarrier = false;

         // sanity check must have at least 1 matching carrier to pass this message
         for(auto carrier : txControl.carriers())
          {
            // check the carrier frequencyHz, see if it matches one of our carrier rx freqs
            if(rxCarriersOfInterest_.count(carrier.frequency_hz()))
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

         statisticManager_.updateRxTableCounts(txControl, rxCarriersOfInterest_);


         pMHAL_->handle_upstream_msg(                               // opaque data
           EMANELTE::MHAL::Data{reinterpret_cast<const char *>(pkt.get()), pkt.length()},

           EMANELTE::MHAL::RxControl{pktInfo.getSource(),           // src nem
               txControl.tx_seqnum(),                               // tx seqnum
               tpToTimeval(tpNow),                                  // clock rx time
               tpToTimeval(pktInfo.getCreationTime()),              // clock tx time
               timeval{txControl.sf_time().ts_sec(),                // sf time / txTime
                       txControl.sf_time().ts_usec()}},

           EMANELTE::MHAL::PHY::OTAInfo{tpTxTime,                   // emulation sot / txTime
               propagationDelay,                                    // propagation delay
               recvInfos},                                          // anteanna rx info
             txControl);                                            // txControl

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
  const auto index = getRxCarrierIndex(carrierFrequencyHz);

  if(index >= 0)
   {
     return messageProcessor_[index]->noiseTestChannelMessage(txControl, channel_msg, segmentCache, carrierFrequencyHz);
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

template <class RadioStatManager, class MessageProcessor>
int EMANE::Models::LTE::RadioModel<RadioStatManager, MessageProcessor>::getRxCarrierIndex(std::uint64_t carrierFrequencyHz) const
{
  const auto iter = rxCarrierFrequencyToIndexTable_.find(carrierFrequencyHz);

  if(iter != rxCarrierFrequencyToIndexTable_.end())
   {
     return iter->second;
   }
  else
   {
     return -1;
   }
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
