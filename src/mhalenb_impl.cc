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


#include "mhalenb_impl.h"
#include "configmanager.h"

void EMANELTE::MHAL::MHALENBImpl::initialize(uint32_t carrierId,
                                             const mhal_config_t & mhal_config,
                                             const ENB::mhal_enb_config_t & mhal_enb_config)
{
  // enb will run thru each carrier each and only once at startup
  if(carrierId == 0)
   {
     // this must be done first one time
     MHALCommon::initialize(mhal_enb_config.subframe_interval_msec_, mhal_config);

     pRadioModel_->setSymbolsPerSlot(mhal_enb_config.symbols_per_slot_);
   }

  const bool clearCache = false;
  const bool searchMode = false;

  physicalCellIds_.insert(mhal_enb_config.physical_cell_id_);

  pRadioModel_->setFrequencies(carrierId,
                               mhal_enb_config.uplink_frequency_hz_,   // rx
                               mhal_enb_config.downlink_frequency_hz_, // tx
                               clearCache);

  pRadioModel_->setNumResourceBlocks(mhal_enb_config.num_resource_blocks_);

  pRadioModel_->setFrequenciesOfInterest(searchMode);
}



void EMANELTE::MHAL::MHALENBImpl::init_emane()
{
  auto & configManager = ConfigManager::getInstance();

  auto & platformConfig = configManager.getPlatformConfig();

  auto & radioModelConfig = configManager.getRadioModelConfig();

  auto & phyConfig = configManager.getPhyConfig();

  // create an NEM builder
  EMANE::Application::NEMBuilder nemBuilder{};

  // create the appropriate radio model, skip config
  auto items = nemBuilder.buildMACLayer_T<EMANE::Models::LTE::ENBRadioModel>(
     platformConfig.id_,
     "lteenbradiomodel",
     {
      {"maxpropagationdelay",  {radioModelConfig.sMaxPropagationDelay_}},
      {"pcrcurveuri",          {radioModelConfig.sPcrCurveURI_}},
      {"resourceblocktxpower", {radioModelConfig.sResourceBlockTxPower_}},
      {"antenna",              {radioModelConfig.sAntenna_}},
      {"subid",                {phyConfig.sSubId_}},
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


void EMANELTE::MHAL::MHALENBImpl::start()
{
  for(auto id : physicalCellIds_)
   {
     logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, physicalCellId_=0x%x", __func__, id);
   }

  MHALCommon::start(5);
}



void EMANELTE::MHAL::MHALENBImpl::send_downstream(const Data & data,
                                                  TxControlMessage & txControl,
                                                  const EMANE::TimePoint & timestamp)
{
  if(pRadioModel_)
    {
      pRadioModel_->sendDownstreamMessage(data, txControl, timestamp);
    }
  else
    {
      logger_.log(EMANE::INFO_LEVEL, " MHALENBImpl %s radiomodel not ready", __func__);
    }
}


void EMANELTE::MHAL::MHALENBImpl::handle_upstream_msg(const Data & data,
                                                      const RxControl & rxControl,
                                                      const PHY::OTAInfo & otaInfo,
                                                      const TxControlMessage & txControl)
{
  MHALCommon::handle_upstream_msg(data, rxControl, otaInfo, txControl);
}


EMANE::SpectrumWindow EMANELTE::MHAL::MHALENBImpl::get_noise(const uint32_t antennaIndex,
                                                             const FrequencyHz frequencyHz, 
                                                             const EMANE::Microseconds & span, 
                                                             const EMANE::TimePoint & sor)
{
  return pRadioModel_->getNoise(antennaIndex, frequencyHz, span, sor);
}


std::uint64_t
EMANELTE::MHAL::MHALENBImpl::get_tx_prb_frequency(const int prb_index, const uint64_t freq_hz)
{
  return pRadioModel_->getTxResourceBlockFrequency(prb_index, freq_hz);
}


void
EMANELTE::MHAL::MHALENBImpl::noise_processor(const uint32_t bin,
                                             const EMANE::Models::LTE::RxAntennaSpectrumWindowCache & rxAntennaSpectrumWindowCache)
{
  const auto carriersOfInterest = pRadioModel_->getCarriersOfInterest();

  // Track the combined transmit power of all UEs on shared channels per recv antenna
  std::map<uint32_t, EMANE::Models::LTE::SegmentSOTMap> antennaInBandSegmentPowerMap_mW;

  // all uplink messages from all ues
  auto & msgs = pendingMessageBins_[bin].get();

  // For the first pass, store the combined receive power for all transmitters 
  // sending on the same frequency segment offset and duration
  for(const auto & msg : msgs)
    {
      const auto & txControl = PendingMessage_TxControl_Get(msg);
      const auto & rxControl = PendingMessage_RxControl_Get(msg);

      if(checkPci_i(txControl) == false)
       {
         logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 1st, sgnore nem %hu, no cells of interest", 
                     __func__, rxControl.nemId_);

         // no cells of interest, skip
         continue;
       }

      const auto & otaInfo = PendingMessage_OtaInfo_Get(msg);

      // enb may have multiple receive paths when carrier aggregation is enabled
      for(const auto & antennaInfo : otaInfo.antennaInfos_)
       { 
         const auto rxAntennaId = antennaInfo.getRxAntennaIndex();

         const auto & frequencySegments = antennaInfo.getFrequencySegments();

         // segment sot map by rx antenna id
         auto & segmentSOTmap = antennaInBandSegmentPowerMap_mW[rxAntennaId];

         for(const auto & frequencySegment : frequencySegments)
          {
            const float rxPower_mW = EMANELTE::DB_TO_MW(frequencySegment.getRxPowerdBm());

            // map key this segment/pulse frequency, offset, duration
            const EMANE::Models::LTE::SegmentKey segmentKeyInBand{frequencySegment.getFrequencyHz(), 
                                                                  frequencySegment.getOffset(),
                                                                  frequencySegment.getDuration()};

            // min/max sot and rx power
            const EMANE::Models::LTE::SegmentSOTValue segmentValueInBand{otaInfo.sot_,
                                                                         otaInfo.sot_,
                                                                         rxPower_mW};

#if 0
            logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 1st, RxAntenna %u, src %hu, contributes segmentKey <%lu, %ld, %ld> power %f dBm", 
                        __func__,
                        rxAntennaId,
                        rxControl.nemId_,
                        std::get<0>(segmentKeyInBand),
                        std::get<1>(segmentKeyInBand).count(),
                        std::get<2>(segmentKeyInBand).count(),
                        frequencySegment.getRxPowerdBm());
#endif

            const auto iter = segmentSOTmap.find(segmentKeyInBand);

            if(iter == segmentSOTmap.end())
             {
               // new entry             
               segmentSOTmap.emplace(segmentKeyInBand, segmentValueInBand);
             }
            else
             {
               // update sot/power
               auto & minSot           (std::get<0>(iter->second));
               auto & maxSot           (std::get<1>(iter->second));
               auto & partialRxPower_mW(std::get<2>(iter->second));
 
               minSot = std::min(minSot, otaInfo.sot_);
               maxSot = std::max(maxSot, otaInfo.sot_);
               partialRxPower_mW += rxPower_mW;
             }
           } // end for each frequency segment
       } // end for each rx antenna
    } // end first pass for all msgs



  // For the second pass, determine the noise floor once all in band segment power is removed.
  // For dedicated channels there should only be 1 in-band transmitter assuming the enb scheduler does not
  // allocate uplink PDSCH resources to more than one UE (an assumption we are making currently).
  // For PUSCH and PRACH, we make an a simplifying assumption for now that inband receptions are time aligned and
  // orthogonal so do not interfere with each other; uplink sinr is calculated for each segment
  // as the segment receive power less the noisefloor of out of band contributions.
  std::map<uint32_t, EMANE::Models::LTE::SegmentMap> antennaOutOfBandNoiseFloorMap_dBm;

  // for each rx antenna
  for(const auto & antennaInBandPower : antennaInBandSegmentPowerMap_mW)
   {
     const auto rxAntennaId = antennaInBandPower.first;

     const auto spectrumWindowCache = rxAntennaSpectrumWindowCache.find(rxAntennaId);

     if(spectrumWindowCache == rxAntennaSpectrumWindowCache.end())
       {
         logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, 2nd, bin %u, no antennaSpectrumWindow cache info for rxAntennaId %u",
                     __func__,
                     bin,
                     rxAntennaId);

         pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(rxAntennaId);

         // something is wrong, not an expected condition
         continue;
       }

     for(auto & segmentSOTMap : antennaInBandPower.second)
      {
        const auto & segmentKeyInBand   = segmentSOTMap.first;
        const auto & segmentValueInBand = segmentSOTMap.second;

        const auto & segmentFrequencyHz = std::get<0>(segmentKeyInBand);
        const auto & offset             = std::get<1>(segmentKeyInBand);
        const auto & duration           = std::get<2>(segmentKeyInBand);

        const auto spectrumWindow = spectrumWindowCache->second.find(segmentFrequencyHz);

        if(spectrumWindow == spectrumWindowCache->second.end())
          {
            logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, 2nd, bin %u, no spectrumWindow cache info for frequency %lu Hz",
                        __func__,
                        bin,
                        segmentFrequencyHz);

            pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

            // something is wrong, not an expected condition
            continue;
          }

        const auto & noiseData = std::get<0>(spectrumWindow->second);

        if(noiseData.empty())
         {
           logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, 2nd, bin %u, freq %lu, no noise data",
                       __func__,
                       bin,
                       segmentFrequencyHz);

           pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

           continue;
         }

        const auto & minSot     = std::get<0>(segmentValueInBand);
        const auto & maxSot     = std::get<1>(segmentValueInBand);
        const auto & rxPower_mW = std::get<2>(segmentValueInBand);

        const auto minSor = minSot + offset;
        const auto maxEor = maxSot + offset + duration;

        // find the max out-of-band noise across the segment bins
        const auto rangeInfo = EMANE::Utils::maxBinNoiseFloorRange(spectrumWindow->second,
                                                                   EMANELTE::MW_TO_DB(rxPower_mW),
                                                                   minSor,
                                                                   maxEor);

        const EMANE::Models::LTE::SegmentKey segmentKeyOutOfBand{segmentFrequencyHz, offset, duration};

        antennaOutOfBandNoiseFloorMap_dBm[rxAntennaId].emplace(segmentKeyOutOfBand, rangeInfo.first);

#if 0
        logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 2nd, RxAntenna %u, add segmentKey <%lu, %ld, %ld> rx power %f dBm, noise %f dB, status %d", 
                    __func__,
                    rxAntennaId,
                    std::get<0>(segmentKeyOutOfBand),
                    std::get<1>(segmentKeyOutOfBand).count(),
                    std::get<2>(segmentKeyOutOfBand).count(),
                    EMANELTE::MW_TO_DB(rxPower_mW),
                    rangeInfo.first,
                    rangeInfo.second);
#endif

      } // end second pass
    }

   
  // now process each message based on rx power from the source per rx antenna and
  // the out of band noise floor
  for(auto & msg : msgs)
    {
      const auto & txControl = PendingMessage_TxControl_Get(msg);

      auto & rxControl = PendingMessage_RxControl_Get(msg);

      if(checkPci_i(txControl) == false)
       {
         logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 3rd, ignore nem %hu, no cells of interest", 
                     __func__, rxControl.nemId_);

         // no cells of interest, skip
         continue;
       }

      const auto & otaInfo = PendingMessage_OtaInfo_Get(msg);

      SINRTesterImpls sinrTesterImpls;

      // enb may have multiple receive paths
      for(const auto & antennaInfo : otaInfo.antennaInfos_)
       { 
         const auto & frequencySegments = antennaInfo.getFrequencySegments();

         const auto rxAntennaId = antennaInfo.getRxAntennaIndex();

         // key frequency, value frequency segments
         std::multimap<std::uint64_t, EMANE::FrequencySegment> frequencySegmentTable;

         // track all segments/subchannels by frequency,
         // may have multiple segments at the same frequency with different slot/offset(s)
         for(const auto & frequencySegment : frequencySegments)
          {
            frequencySegmentTable.emplace(frequencySegment.getFrequencyHz(), frequencySegment);
          }

         // for each carrier typically expect only 1 with omni antenna for ue
         // CA may add more carriers in the future
         for(const auto & txCarrier : txControl.carriers())
          {
            // txCarrier center freq 
            const auto txCarrierFrequencyHz = txCarrier.frequency_hz();

            // txAntenna is directly related to txAntennaId
            const auto txAntennaId   = txCarrier.carrier_id();

            // frequency segments for this carrier
            EMANE::FrequencySegments frequencySegmentsTxAntenna;

            // for each sub channel of the carrier
            for(const auto subChannelHz : txCarrier.sub_channels())
             {
               if(frequencySegmentTable.count(subChannelHz))
                {
                  const auto range = frequencySegmentTable.equal_range(subChannelHz);
 
                  // get all segment(s) matching this subchannel frequency
                  for(auto iter = range.first; iter != range.second; ++iter)
                   {
                     frequencySegmentsTxAntenna.push_back(iter->second);
                   }
                }
               else
                {
                  logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 3rd, src %hu, rxAntennaId %u, txAntenna %u, missing subchannel %lu",
                              __func__,
                              rxControl.nemId_,
                              rxAntennaId,
                              txAntennaId,
                              subChannelHz);
                }
             }

            if(frequencySegmentsTxAntenna.empty())
             {
               logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, 3rd, src %hu, missing all subChannels, skip rxAntennaId %u, txAntennaId %u, txFrequency %lu",
                           __func__,
                           rxControl.nemId_,
                           rxAntennaId, 
                           txAntennaId,
                           txCarrierFrequencyHz);
 
               // can not reasemble msg, skip
               continue;
             }

            const auto spectrumWindowCache = rxAntennaSpectrumWindowCache.find(rxAntennaId);

            if(spectrumWindowCache == rxAntennaSpectrumWindowCache.end())
             {
               logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, 3rd, bin %u, no antennaSpectrumWindow cache info for rxAntennaId %u",
                           __func__,
                           bin,
                           rxAntennaId);
  
               pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(rxAntennaId);
 
               // something is wrong, not an expected condition
               continue;
             }

           float signalSum_mW     = 0.0,
                 noiseFloorSum_mW = 0.0,
                 peakSum          = 0.0;

           EMANE::Models::LTE::SegmentMap segmentCache;

           // build segmentCache
           for(const auto & frequencySegment : frequencySegmentsTxAntenna)
            {
              const auto segmentFrequencyHz = frequencySegment.getFrequencyHz();
              const auto spectrumWindow     = spectrumWindowCache->second.find(segmentFrequencyHz);

              if(spectrumWindow == spectrumWindowCache->second.end())
               {
                 logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s 3rd, src %hu, bin %u, no spectrumWindow cache info for freq %lu",
                             __func__,
                             rxControl.nemId_,
                             bin,
                             segmentFrequencyHz);
 
                 pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segmentFrequencyHz);

                 // something is wrong, not an expected condition
                 continue;
               }

              const auto rxPower_dBm = frequencySegment.getRxPowerdBm();
              const auto rxPower_mW  = EMANELTE::DB_TO_MW(rxPower_dBm);

              const auto antennaOutOfBandIter = antennaOutOfBandNoiseFloorMap_dBm.find(rxAntennaId);

              if(antennaOutOfBandIter == antennaOutOfBandNoiseFloorMap_dBm.end())
               {
                 logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s 3rd, src %hu, no out out band noise entry for rxAntenna %u",
                             __func__,
                             rxControl.nemId_,
                             rxAntennaId);
 
                 // something is wrong, not an expected condition
                 continue;
               }
 
              const EMANE::Models::LTE::SegmentKey segmentKeyOutOfBand{segmentFrequencyHz,
                                                              frequencySegment.getOffset(), 
                                                              frequencySegment.getDuration()};
 
              const auto outOfBandNoiseIter = antennaOutOfBandIter->second.find(segmentKeyOutOfBand);
 
              if(outOfBandNoiseIter == antennaOutOfBandIter->second.end())
               {
                 logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s 3rd, src %hu, no out out band noise info for rxAntenna %u, <%lu, %ld, %ld>",
                             __func__,
                             rxControl.nemId_,
                             rxAntennaId,
                             segmentFrequencyHz,
                             frequencySegment.getOffset().count(),
                             frequencySegment.getDuration().count());
#if 1 
                 for(const auto & e : antennaOutOfBandIter->second)
                  {
                    logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s 3rd, candidate segmentKey<%lu, %ld, %ld>",
                                __func__, 
                                std::get<0>(e.first),
                                std::get<1>(e.first).count(),
                                std::get<2>(e.first).count());
                  }
#endif

                 // something is wrong, not an expected condition
                 continue;
               }
            
              const auto noiseFloor_dBm = outOfBandNoiseIter->second;
              const auto noiseFloor_mW  = EMANELTE::DB_TO_MW(noiseFloor_dBm);
              const auto sinr_dB        = rxPower_dBm - noiseFloor_dBm;

#if 0 
              logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s 3rd, src %hu, found rxAntenna %u, <%lu, %ld, %ld>, noise %f dBm, sinr %f dB",
                          __func__,
                          rxControl.nemId_,
                          rxAntennaId,
                          segmentFrequencyHz,
                          frequencySegment.getOffset().count(),
                          frequencySegment.getDuration().count(),
                          noiseFloor_dBm,
                          sinr_dB);
#endif
  
              signalSum_mW     += rxPower_mW;
              noiseFloorSum_mW += noiseFloor_mW;
 
              segmentCache.emplace(segmentKeyOutOfBand, sinr_dB);
 
              pRadioModel_->getStatisticManager().updateRxFrequencyAvgNoiseFloor(segmentFrequencyHz, noiseFloor_mW);
 
              peakSum += sinr_dB;
            } // end each segment

          const auto segmentCacheSize = segmentCache.size();

#if 0
          logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s 3rd, src %hu, rxAntennaId %u, txAntennaId %u, txFrequency %lu, segmentCacheSize %zu",
                      __func__, rxControl.nemId_, rxAntennaId, txAntennaId, txCarrierFrequencyHz, segmentCacheSize);
#endif

          // now check for number of pass/fail segments
          if(segmentCacheSize > 0)
           {
             const auto signalAvg_dBm     = EMANELTE::MW_TO_DB(signalSum_mW     / segmentCacheSize);
             const auto noiseFloorAvg_dBm = EMANELTE::MW_TO_DB(noiseFloorSum_mW / segmentCacheSize);

             auto pSINRTester = new UplinkSINRTesterImpl(signalAvg_dBm - noiseFloorAvg_dBm,
                                                         noiseFloorAvg_dBm,
                                                         txCarrierFrequencyHz,
                                                         txAntennaId);

             // txCarrierFrequency, rxAntennaId, txAntennaId
             sinrTesterImpls[SINRTesterKey(txCarrierFrequencyHz, rxAntennaId, txAntennaId)].reset(pSINRTester);

             rxControl.peak_sum_   [txAntennaId] = peakSum;
             rxControl.num_samples_[txAntennaId] = segmentCacheSize;
             rxControl.avg_snr_    [txAntennaId] = signalAvg_dBm - noiseFloorAvg_dBm;
             rxControl.avg_nf_     [txAntennaId] = noiseFloorAvg_dBm;
             rxControl.is_valid_   [txAntennaId] = true;

             if(txCarrier.uplink().has_prach())
              {
                const auto & prach = txCarrier.uplink().prach();

                putSINRResult_i(prach,
                                rxControl,
                                pSINRTester,
                                pRadioModel_->noiseTestChannelMessage(txControl, 
                                                                      prach,
                                                                      segmentCache,
                                                                      txCarrierFrequencyHz,
                                                                      txAntennaId));
              }

             for(const auto & pucch : txCarrier.uplink().pucch())
              {
                putSINRResult_i(pucch,
                                rxControl,
                                pSINRTester,
                                pRadioModel_->noiseTestChannelMessage(txControl,
                                                                      pucch,
                                                                      segmentCache,
                                                                      txCarrierFrequencyHz,
                                                                      txAntennaId));
              }

             for(const auto & pusch : txCarrier.uplink().pusch())
              {
                putSINRResult_i(pusch,
                                rxControl,
                                pSINRTester,
                                pRadioModel_->noiseTestChannelMessage(txControl,
                                                                      pusch,
                                                                      segmentCache,
                                                                      txCarrierFrequencyHz,
                                                                      txAntennaId));
              }

             StatisticManager::ReceptionInfoMap receptionInfoMap;

             // add entry per src
             receptionInfoMap[rxControl.nemId_] = StatisticManager::ReceptionInfoData{signalAvg_dBm,
                                                                                      noiseFloorAvg_dBm,
                                                                                      segmentCacheSize};
             statisticManager_.updateReceptionTable(receptionInfoMap);
           }         
        } // end tx each carrier
      } // end for each rx antenna

     if(! sinrTesterImpls.empty())
      {
        // make ready for phy_adapter use
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


void
EMANELTE::MHAL::MHALENBImpl::putSINRResult_i(const ChannelMessage & channel_message,
                                             const RxControl & rxControl __attribute__((unused)),
                                             UplinkSINRTesterImpl * pSINRTester,
                                             const bool received)
{
  CHANNEL_TYPE ctype = channel_message.channel_type();

  if(channel_message.has_rnti())
    {
      pSINRTester->rntiChannelSINRResults_.emplace(ChannelRNTI(ctype, channel_message.rnti()), received);
    }
  else
    {
      pSINRTester->channelSINRResults_.emplace(ctype, received);
    }
}



bool 
EMANELTE::MHAL::MHALENBImpl::checkPci_i(const TxControlMessage & txControl) const
{
  for(const auto & carrier : txControl.carriers())
   {
     if(physicalCellIds_.count(carrier.phy_cell_id()))
      {
        // pci of interest found
        return true;
      }
   }

  return false;
}
