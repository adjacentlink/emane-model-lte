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


#include "mhalue_impl.h"
#include "configmanager.h"
#include "segmentmap.h"


void EMANELTE::MHAL::MHALUEImpl::initialize(uint32_t sfIntervalMilliseconds,
                                            const mhal_config_t & mhalConfig)
{
  MHALCommon::initialize(sfIntervalMilliseconds, mhalConfig);

  pRadioModel_->setSubframeInterval(sfIntervalMilliseconds);
}


void EMANELTE::MHAL::MHALUEImpl::init_emane()
{
  auto & configManager = ConfigManager::getInstance();

  auto & platformConfig = configManager.getPlatformConfig();

  auto & radioModelConfig = configManager.getRadioModelConfig();

  auto & phyConfig = configManager.getPhyConfig();

  // create an NEM builder
  EMANE::Application::NEMBuilder nemBuilder{};

  // create the appropriate radio model, skip config
  auto items = nemBuilder.buildMACLayer_T<EMANE::Models::LTE::UERadioModel>(
        platformConfig.id_,
        "lteueradiomodel",
        {
          {"maxpropagationdelay",  {radioModelConfig.sMaxPropagationDelay_}},
          {"pcrcurveuri",          {radioModelConfig.sPcrCurveURI_}},
          {"resourceblocktxpower", {radioModelConfig.sResourceBlockTxPower_}},
          {"subid",                {phyConfig.sSubId_}}
        },
        false);

  pRadioModel_ = std::get<0>(items);

  pRadioModel_->setMHAL(this);

  // create an NEMLayers instance to hold the layers of your NEM
  EMANE::Application::NEMLayers layers{};

  // add your radio model (as an NEMLayer) to your NEM layers
  layers.push_back(std::move(std::get<1>(items)));

  // create and add to layers an emulator physical layer instance
  layers.push_back(nemBuilder.buildPHYLayer(platformConfig.id_,
                                            "",
                                            {
                                              {"fixedantennagain",       {phyConfig.sAntennaGain_}},
                                              {"fixedantennagainenable", {phyConfig.sFixedAntennaGainEnable_}},
                                              {"noisemode",              {phyConfig.sNoiseMode_}},
                                              {"propagationmodel",       {phyConfig.sPropagationModel_}},
                                              {"systemnoisefigure",      {phyConfig.sSystemNoiseFigure_}},
                                              {"subid",                  {phyConfig.sSubId_}},
                                              {"compatibilitymode",      {phyConfig.sCompatibilityMode_}}
                                            },
                                            false)); // skip config

  nems_.push_back(nemBuilder.buildNEM(platformConfig.id_,
                                      layers,
                                      {},
                                      false));

  // create application instance UUID
  uuid_t uuid;
  uuid_generate(uuid);

  pNEMManager_ =
    nemBuilder.buildNEMManager(uuid,
                               nems_,
                               {
                                 {"otamanagerchannelenable",   {platformConfig.sOtamManagerChannelEnable_}},
                                 {"otamanagergroup",           {platformConfig.sOtaManagerGroup_}},
                                 {"otamanagerloopback",        {platformConfig.sOtaManagerLoopback_}},
                                 {"otamanagerdevice",          {platformConfig.sOtaManagerDevice_}},
                                 {"eventservicegroup",         {platformConfig.sEventServiceGroup_}},
                                 {"eventservicedevice",        {platformConfig.sEventServiceDevice_}},
                                 {"controlportendpoint",       {platformConfig.sControlPortEndpoint_}},
                                 {"antennaprofilemanifesturi", {platformConfig.sAntennaProfileManifest_}}
                               });

  pNEMManager_->start();

  pNEMManager_->postStart();
}


void EMANELTE::MHAL::MHALUEImpl::start()
{
  MHALCommon::start(0);
}


void EMANELTE::MHAL::MHALUEImpl::send_downstream(const Data & data,
                                                 TxControlMessage & txControl,
                                                 const EMANE::TimePoint & timestamp)
{
  if(pRadioModel_)
    {
      pRadioModel_->sendDownstreamMessage(data, txControl, timestamp);
    }
  else
    {
      logger_.log(EMANE::INFO_LEVEL, " EMANELTE::MHAL::MHALENBImpl %s radiomodel not ready", __func__);
    }
}


void EMANELTE::MHAL::MHALUEImpl::handle_upstream_msg(const Data & data,
                                                     const RxControl & rxControl,
                                                     const PHY::OTAInfo & otaInfo,
                                                     const TxControlMessage & txControl)
{
  // ue starts with cell search, until then receiver is off
  if(!state_.ue_init())
    {
      logger_.log(EMANE::INFO_LEVEL, " MHAL::PHY %s ue not in cell search yet, discard", __func__);

      return;
    }

  MHALCommon::handle_upstream_msg(data, rxControl, otaInfo, txControl);
}


EMANE::SpectrumWindow EMANELTE::MHAL::MHALUEImpl::get_noise(const uint32_t antennaIndex,
                                                            const FrequencyHz frequencyHz, 
                                                            const EMANE::Microseconds & span, 
                                                            const EMANE::TimePoint & sor)
{
  return pRadioModel_->getNoise(antennaIndex, frequencyHz, span, sor);
}


void
EMANELTE::MHAL::MHALUEImpl::cell_search()
{
  logger_.log(EMANE::INFO_LEVEL, "MHAL %s", __func__);

  for(size_t bin = 0; bin < EMANELTE::NUM_SF_PER_FRAME; ++bin)
   {
     pendingMessageBins_[bin].lockBin();
     pendingMessageBins_[bin].clear();
     readyMessageBins_  [bin].clear();
     pendingMessageBins_[bin].unlockBin();
   }

  timing_.lockTime();

  timing_.alignTime();
  state_.ue_init(true);

  timing_.unlockTime();
}


void
EMANELTE::MHAL::MHALUEImpl::set_frequencies(const uint32_t cc_idx,
                                            const uint32_t cellid,
                                            const bool scell,
                                            const FrequencyHz carrierRxFrequencyHz,
                                            const FrequencyHz carrierTxFrequencyHz)
{
  const bool searchMode = (scell == false) && (cc_idx == 0);

  logger_.log(EMANE::INFO_LEVEL, "MHAL %s cc=%u, cellid %u, searchMode %d, rxFreq %lu, txFreq %lu",
              __func__, cc_idx, cellid, searchMode, carrierRxFrequencyHz, carrierTxFrequencyHz);


  pRadioModel_->setFrequencies(cc_idx,                // carrier idx
                               carrierRxFrequencyHz,  // rx freq
                               carrierTxFrequencyHz,  // tx freq
                               searchMode);           // clear cache

  if(searchMode)
   {
     logger_.log(EMANE::INFO_LEVEL, "MHAL %s reset num_prb to 6", __func__);

     // set to full bandwidth during cell search, 
     // when the mib is detected, the bandwidth will be adjusted
     pRadioModel_->setNumResourceBlocks(6);
   }

  pRadioModel_->setFrequenciesOfInterest(searchMode);
}


void
EMANELTE::MHAL::MHALUEImpl::set_num_resource_blocks(const int numResourceBlocks)
{
  logger_.log(EMANE::INFO_LEVEL, "MHAL %s numResouresBlocks %d ", __func__, numResourceBlocks);

  // called when bandwidth is detected
  pRadioModel_->setNumResourceBlocks(numResourceBlocks);

  pRadioModel_->setFrequenciesOfInterest(false); // search mode false
}


EMANELTE::FrequencyHz
EMANELTE::MHAL::MHALUEImpl::get_tx_prb_frequency(const int prbIndex, const FrequencyHz channelFrequencyHz)
{
  return pRadioModel_->getTxResourceBlockFrequency(prbIndex, channelFrequencyHz);
}


void
EMANELTE::MHAL::MHALUEImpl::noise_processor(const uint32_t bin,
                                            const EMANE::Models::LTE::RxAntennaSpectrumWindowCache & rxAntennaSpectrumWindowCache)
{
  const auto carriersOfInterest = pRadioModel_->getCarriersOfInterest();

  // for each enb dl pending msg
  for(auto & msg : pendingMessageBins_[bin].get())
   {
     // get the ota info
     const auto & otaInfo = PendingMessage_OtaInfo_Get(msg);

     // get the txControl info
     const auto & txControl = PendingMessage_TxControl_Get(msg);

     // ue supports 1 antenna
     const uint32_t rxAntennaId = 0;

     // get the rxControl info
     auto & rxControl = PendingMessage_RxControl_Get(msg);

     SINRTesterImpls sinrTesterImpls;

     //<txAntenna> <frequency, frequency_segments>
     std::map<uint32_t, std::multimap<std::uint64_t, EMANE::FrequencySegment>> antennaFrequencySegmentTable;

     // ue has 1 antenna, but we can receive from more than 1 enb tx antenna
     for(const auto & antennaInfo : otaInfo.antennaInfos_)
      {
        // txAntenna is directly related to txCarrierId
        const auto txAntennaId = antennaInfo.getTxAntennaIndex();

        const auto & frequencySegments = antennaInfo.getFrequencySegments();

#if 0
        logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, rxAntennaId %u, txAntennaId %u, num segments %zu",
                    __func__,
                    rxControl.nemId_,
                    rxAntennaId,
                    txAntennaId,
                    frequencySegments.size());
#endif

        // track all segments/subchannels by frequency,
        // may have multiple segments at the same frequency with different slot/offset(s)
        for(auto & segment : frequencySegments)
         {
           antennaFrequencySegmentTable[txAntennaId].emplace(segment.getFrequencyHz(), segment);
#if 0
           logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, rxAntennaId %u, txAntennaId %u, segment[frequency %lu Hz, offset %ld, duration %ld]",
                       __func__,
                      rxControl.nemId_,
                      rxAntennaId,
                      txAntennaId,
                      segment.getFrequencyHz(),
                      segment.getOffset().count(),
                      segment.getDuration().count());
#endif
         }
      }

     // for each carrier
     for(const auto & carrier : txControl.carriers())
      {
        // carrier center freq and carried id
        const auto txFrequencyHz = carrier.frequency_hz();
        const auto txCarrierId   = carrier.carrier_id();

        // check that we do have recv antenna info for this txAntenna / carrrierId
        if(antennaFrequencySegmentTable.count(txCarrierId) == 0)
         {
#if 0
           logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, txFrequencyHz %lu, txCarrierId %u, no segemnts, skip",
                       __func__,
                      rxControl.nemId_,
                      txFrequencyHz,
                      txCarrierId);
#endif

           // no segments, skip
           continue;
         }

        // frequency segments for this carrier
        EMANE::FrequencySegments segmentsThisCarrier;

        // for each sub channel of the carrier
        for(const auto subChannelHz : carrier.sub_channels())
         {
           if(antennaFrequencySegmentTable[txCarrierId].count(subChannelHz))
            {
              const auto range = antennaFrequencySegmentTable[txCarrierId].equal_range(subChannelHz);
 
              // get all segment(s) matching this subchannel frequency
              for(auto iter = range.first; iter != range.second; ++iter)
               {
                 segmentsThisCarrier.push_back(iter->second);
#if 0
                 logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, rxAntennaId %u, txCarrierId %u, txFrequency %lu Hz, subchannel %lu Hz, segment[frequency %lu Hz, offset %ld, duration %ld]",
                             __func__,
                             rxControl.nemId_,
                             rxAntennaId,
                             txCarrierId,
                             txFrequencyHz,
                             subChannelHz,
                             iter->second.getFrequencyHz(),
                             iter->second.getOffset().count(),
                             iter->second.getDuration().count());
#endif
               }
            }
           else
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, rxAntennaId %u, txCarrierId %u, txFrequency %lu Hz, subchannel %lu Hz, not found",
                          __func__,
                          rxControl.nemId_,
                          rxAntennaId,
                          txCarrierId,
                          txFrequencyHz,
                          subChannelHz);
            }
         }

        // check for missing all segments
        if(segmentsThisCarrier.empty())
         {
           logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, missing all subChannels, skip txFrequency %lu",
                       __func__, txFrequencyHz);

           // can not reasemble any msg(s), skip
           continue;
         }

        float signalSum_mW     = 0.0, 
              noiseFloorSum_mW = 0.0, 
              peakSum          = 0.0;

        EMANE::Models::LTE::SegmentMap segmentCache;

        // build segmentCache based on actual received segments for this carrier
        for(auto & segment : segmentsThisCarrier)
         {
           const auto segmentFrequencyHz = segment.getFrequencyHz();

           const auto rxAntennaSpectrum = rxAntennaSpectrumWindowCache.find(rxAntennaId);

           if(rxAntennaSpectrum == rxAntennaSpectrumWindowCache.end())
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, no spectrumWindow cache info for txCarrierId %u, rxAntennaId %u",
                          __func__,
                          rxControl.nemId_,
                          bin,
                          txCarrierId,
                          rxAntennaId);

              pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

              // something is wrong, not an expected condition
              continue;
            }

           const auto spectrumWindow = rxAntennaSpectrum->second.find(segmentFrequencyHz);

           if(spectrumWindow == rxAntennaSpectrum->second.end())
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, no spectrumWindow cache info for frequency %lu",
                          __func__,
                          rxControl.nemId_,
                          bin,
                          segmentFrequencyHz);

              pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

              // something is wrong, not an expected condition
              continue;
            }

           const auto & noiseData = std::get<0>(spectrumWindow->second);

           if(noiseData.empty())
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, frequency %lu, no noise data",
                          __func__,
                          rxControl.nemId_,
                          bin,
                          segmentFrequencyHz);

              pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

              // something is wrong, not an expected condition
              continue;
            }

           const auto rxPower_dBm = segment.getRxPowerdBm();
           const auto rxPower_mW  = EMANELTE::DB_TO_MW(segment.getRxPowerdBm());

           const auto segmentSor = otaInfo.sot_ + segment.getOffset();
           const auto segmentEor = segmentSor   + segment.getDuration();

           const auto rangeInfo  = EMANE::Utils::maxBinNoiseFloorRange(spectrumWindow->second, rxPower_dBm, segmentSor, segmentEor);

           // HACK force low noise floor to allow for multiple cell testing
           const bool celltest = false;
           const auto noiseFloor_dBm = celltest ? -111.0f : rangeInfo.first;
           const auto noiseFloor_mW  = EMANELTE::DB_TO_MW(noiseFloor_dBm);

           const auto sinr_dB = rxPower_dBm - noiseFloor_dBm;

           signalSum_mW     += rxPower_mW;
           noiseFloorSum_mW += noiseFloor_mW;

           segmentCache.emplace(EMANE::Models::LTE::SegmentKey{segmentFrequencyHz, segment.getOffset(), segment.getDuration()}, sinr_dB);

           pRadioModel_->getStatisticManager().updateRxFrequencyAvgNoiseFloor(segmentFrequencyHz, noiseFloor_mW);

           peakSum += sinr_dB;
         } // end each segment

         const auto segmentCacheSize = segmentCache.size();

         // now check for number of pass/fail segments
         if(segmentCacheSize > 0)
          {
            const auto signalAvg_dBm     = EMANELTE::MW_TO_DB(signalSum_mW     / segmentCacheSize);
            const auto noiseFloorAvg_dBm = EMANELTE::MW_TO_DB(noiseFloorSum_mW / segmentCacheSize);

            const bool pcfichPass = pRadioModel_->noiseTestChannelMessage(txControl, 
                                                                          carrier.downlink().pcfich(), 
                                                                          segmentCache,
                                                                          txFrequencyHz,
                                                                          txCarrierId);

            const bool pbchPass = carrier.downlink().has_pbch() ?
                                    pRadioModel_->noiseTestChannelMessage(txControl, 
                                                                          carrier.downlink().pbch(),
                                                                          segmentCache,
                                                                          txFrequencyHz,
                                                                          txCarrierId) : false;

            auto pSINRTester = new DownlinkSINRTesterImpl(pRadioModel_, 
                                                          txControl,
                                                          segmentCache, 
                                                          pcfichPass,
                                                          pbchPass,
                                                          signalAvg_dBm - noiseFloorAvg_dBm,
                                                          noiseFloorAvg_dBm,
                                                          txFrequencyHz,
                                                          txCarrierId);

            // carrierFrequency, rxAntennaId, txCarrierId
            sinrTesterImpls[SINRTesterKey(txFrequencyHz, rxAntennaId, txCarrierId)].reset(pSINRTester);

            // load carrier info
            rxControl.peak_sum_   [txCarrierId] = peakSum;
            rxControl.num_samples_[txCarrierId] = segmentCacheSize;
            rxControl.avg_snr_    [txCarrierId] = signalAvg_dBm - noiseFloorAvg_dBm;
            rxControl.avg_nf_     [txCarrierId] = noiseFloorAvg_dBm;
            rxControl.is_valid_   [txCarrierId] = true;

            StatisticManager::ReceptionInfoMap receptionInfoMap;

            // add entry per src
            receptionInfoMap[rxControl.nemId_] = StatisticManager::ReceptionInfoData{signalAvg_dBm,
                                                                                     noiseFloorAvg_dBm,
                                                                                     segmentCacheSize};
  
            statisticManager_.updateReceptionTable(receptionInfoMap);
         }
       } // end each carrier

      if(! sinrTesterImpls.empty())
       {
         // make ready for phy_adater use
         readyMessageBins_[bin].get().emplace_back(RxMessage{PendingMessage_Data_Get(msg),
                                                             rxControl,
                                                             sinrTesterImpls});
       }
      else
       {
         logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, no sinr testers loaded", __func__);
       }
    } // end for each msg
}
