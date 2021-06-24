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


void EMANELTE::MHAL::MHALUEImpl::initialize(uint32_t sf_interval_msec,
                                            const mhal_config_t & mhal_config)
{
  MHALCommon::initialize(sf_interval_msec, mhal_config);

  pRadioModel_->setSubframeInterval(sf_interval_msec);
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
EMANELTE::MHAL::MHALUEImpl::begin_cell_search()
{
  logger_.log(EMANE::INFO_LEVEL, "MHAL %s", __func__);

  // on cell search, configure for the largest bandwidth to
  // ensure packet reception from any enb (register all possible FOI
  // for the configured receive frequency)
  pRadioModel_->setNumResourceBlocks(100);

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
EMANELTE::MHAL::MHALUEImpl::set_frequencies(uint32_t carrierIndex, float carrierRxFrequencyHz, float carrierTxFrequencyHz)
{
  // ue will rotate thru its frequency list using carrierIndex 0 initially
  // if/when the carrierIndex is > 0, then accumulate frequencies
  const bool clearCache = carrierIndex == 0;
  const bool searchMode = carrierIndex == 0;

  pRadioModel_->setFrequencies(carrierIndex,                   // carrier idx
                               llround(carrierRxFrequencyHz),  // rx freq
                               llround(carrierTxFrequencyHz),  // tx freq
                               clearCache);                    // clear cache

  if(carrierIndex == 0)
   {
     // when the mib is detected, the bandwidth will be adjusted
     pRadioModel_->setNumResourceBlocks(100);
   }

  pRadioModel_->setFrequenciesOfInterest(searchMode);
}


void
EMANELTE::MHAL::MHALUEImpl::set_num_resource_blocks(int numResourceBlocks)
{
  // called when bandwidth is detected
  pRadioModel_->setNumResourceBlocks(numResourceBlocks);

  pRadioModel_->setFrequenciesOfInterest(false);
}


std::uint64_t
EMANELTE::MHAL::MHALUEImpl::get_tx_prb_frequency(int prb_index, std::uint64_t channelFrequencyHz)
{
  return pRadioModel_->getTxResourceBlockFrequency(prb_index, channelFrequencyHz);
}


void
EMANELTE::MHAL::MHALUEImpl::noise_processor(const uint32_t bin,
                                            const EMANE::Models::LTE::AntennaSpectrumWindowCache & antennaSpectrumWindowCache)
{
  const auto carriersOfInterest = pRadioModel_->getCarriersOfInterest();

  // for each enb dl pending msg
  for(auto & msg : pendingMessageBins_[bin].get())
   {
     // get the ota info
     const auto & otaInfo = PendingMessage_OtaInfo_Get(msg);

     // get the txControl info
     const auto & txControl = PendingMessage_TxControl_Get(msg);

     // get the rxControl info
     auto & rxControl = PendingMessage_RxControl_Get(msg);

     SINRTesterImpls sinrTesterImpls;

     //<tx_antenna_index> <frequency, frequency_segments>
     std::map<uint32_t, std::multimap<std::uint64_t, EMANE::FrequencySegment>> antennaFrequencySegmentTable;

     // ue has 1 antenna, but we can receive from more than 1 enb antenna
     for(const auto & antennaInfo : otaInfo.antennaInfos_)
      {
        // tracking txAntenna == carrierId
        const auto txAntennaIndex = antennaInfo.getTxAntennaIndex();

        const auto & frequencySegments = antennaInfo.getFrequencySegments();

        // track all segments/subchannels by frequency,
        // may have multiple segments at the same frequency with different slot/offset(s)
        for(auto & segment : frequencySegments)
         {
           antennaFrequencySegmentTable[txAntennaIndex].emplace(segment.getFrequencyHz(), segment);
         }
      }

     // for each carrier
     for(const auto & carrier : txControl.carriers())
      {
        // carrier center freq and carried id
        const auto carrierFrequencyHz = carrier.frequency_hz();
        const auto carrierId          = carrier.carrier_id();

        // local carrierId
        const auto localCarrierId = pRadioModel_->getRxCarrierIndex(carrierFrequencyHz);

        // check carriers of interest
        if(localCarrierId < 0 || carriersOfInterest.count(carrierFrequencyHz) == 0)
         {
           logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, ignore carrier frequency %lu Hz, not of interest",
                       __func__,
                      rxControl.nemId_,
                      carrierFrequencyHz);
           
           continue;
         }

        // check that we did recv rx info for this txAntenna / carrrierId
        if(antennaFrequencySegmentTable.count(carrierId) == 0)
         {
           continue;
         }

        // frequency segments for this carrier
        EMANE::FrequencySegments segmentsThisCarrier;

        // for each sub channel of the carrier
        for(const auto subChannel : carrier.sub_channels())
         {
           if(antennaFrequencySegmentTable[carrierId].count(subChannel))
            {
              const auto range = antennaFrequencySegmentTable[carrierId].equal_range(subChannel);
 
              // get all segment(s) matching this subchannel frequency
              for(auto iter = range.first; iter != range.second; ++iter)
               {
                 segmentsThisCarrier.push_back(iter->second);
               }
            }
         }

         // check for missing all segments
         if(segmentsThisCarrier.empty())
          {
            continue;
          }

         float signalSum_mW     = 0.0, 
               noiseFloorSum_mW = 0.0, 
               peakSum          = 0.0;

         EMANE::Models::LTE::SegmentMap segmentCache;

         // build segmentCache based on actual received segments for this carrier
         for(auto & segment : segmentsThisCarrier)
          {
            const auto frequencyHz = segment.getFrequencyHz();

            // ue only has 1 antenna
            const auto antennaSpectrum = antennaSpectrumWindowCache.find(0);

            if(antennaSpectrum == antennaSpectrumWindowCache.end())
             {
               logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, no spectrumWindow cache info for carrierId %u, on rxAntenna 0",
                           __func__,
                           rxControl.nemId_,
                           bin,
                           carrierId);

               pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(frequencyHz);

               continue;
             }

            const auto spectrumWindow = antennaSpectrum->second.find(frequencyHz);

            if(spectrumWindow == antennaSpectrum->second.end())
             {
               logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, no spectrumWindow cache info for freq %lu",
                           __func__,
                           rxControl.nemId_,
                           bin,
                           frequencyHz);

               pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(frequencyHz);

               continue;
             }

            const auto & noiseData = std::get<0>(spectrumWindow->second);

            if(noiseData.empty())
             {
               logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, freq %lu, no noise data",
                           __func__,
                           rxControl.nemId_,
                           bin,
                           frequencyHz);

               pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segment.getFrequencyHz());

               continue;
             }

            const auto rxPower_dBm = segment.getRxPowerdBm();
            const auto rxPower_mW  = EMANELTE::DB_TO_MW(segment.getRxPowerdBm());

            const auto segmentSor = otaInfo.sot_ + segment.getOffset();
            const auto segmentEor = segmentSor   + segment.getDuration();
            const auto rangeInfo  = EMANE::Utils::maxBinNoiseFloorRange(spectrumWindow->second, rxPower_dBm, segmentSor, segmentEor);

            const auto noiseFloor_dBm = rangeInfo.first;
            const auto noiseFloor_mW  = EMANELTE::DB_TO_MW(noiseFloor_dBm);

            const auto sinr_dB = rxPower_dBm - noiseFloor_dBm;

            signalSum_mW     += rxPower_mW;
            noiseFloorSum_mW += noiseFloor_mW;

            segmentCache.emplace(EMANE::Models::LTE::SegmentKey{frequencyHz, segment.getOffset(), segment.getDuration()}, sinr_dB);

            pRadioModel_->getStatisticManager().updateRxFrequencyAvgNoiseFloor(frequencyHz, noiseFloor_mW);

            peakSum += sinr_dB;
          } // end each segment

          const auto segmentCacheSize = segmentCache.size();

#if 0
          logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, carrierId %u, frequency %lu, segmentCacheSize %zu",
                      __func__, rxControl.nemId_, carrierId, carrierFrequencyHz, segmentCacheSize);
#endif

          // now check for number of pass/fail segments
          if(segmentCacheSize > 0)
           {
             const auto signalAvg_dBm     = EMANELTE::MW_TO_DB(signalSum_mW     / segmentCacheSize);
             const auto noiseFloorAvg_dBm = EMANELTE::MW_TO_DB(noiseFloorSum_mW / segmentCacheSize);

             const bool pcfichPass = pRadioModel_->noiseTestChannelMessage(txControl, 
                                                                           carrier.downlink().pcfich(), 
                                                                           segmentCache,
                                                                           carrierFrequencyHz);

             const bool pbchPass = carrier.downlink().has_pbch() ?
                                     pRadioModel_->noiseTestChannelMessage(txControl, 
                                                                           carrier.downlink().pbch(),
                                                                           segmentCache,
                                                                           carrierFrequencyHz) : false;

             auto pSINRtester = new DownlinkSINRTesterImpl(pRadioModel_, 
                                                           txControl,
                                                           segmentCache, 
                                                           pcfichPass,
                                                           pbchPass,
                                                           signalAvg_dBm - noiseFloorAvg_dBm,
                                                           noiseFloorAvg_dBm,
                                                           carrierFrequencyHz);

             // sinr tester mapped by carrierId
             sinrTesterImpls[SINRTesterKey(carrierFrequencyHz, carrierId)].reset(pSINRtester);

             // load carrier info
             rxControl.peak_sum_   [carrierId] = peakSum;
             rxControl.num_samples_[carrierId] = segmentCacheSize;
             rxControl.avg_snr_    [carrierId] = signalAvg_dBm - noiseFloorAvg_dBm;
             rxControl.avg_nf_     [carrierId] = noiseFloorAvg_dBm;
             rxControl.is_valid_   [carrierId] = true;

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
         // lastly, make ready
         readyMessageBins_[bin].get().emplace_back(RxMessage{PendingMessage_Data_Get(msg),
                                                             rxControl,
                                                             sinrTesterImpls});
       }
    } // end for each msg
}
