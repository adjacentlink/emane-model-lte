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


void EMANELTE::MHAL::MHALENBImpl::initialize(const mhal_config_t & mhal_config,
                                             const ENB::mhal_enb_config_t & mhal_enb_config)
{
  MHALCommon::initialize(mhal_enb_config.subframe_interval_msec_, mhal_config);

  physicalCellId_ = mhal_enb_config.physical_cell_id_;

  pRadioModel_->setSymbolsPerSlot(mhal_enb_config.symbols_per_slot_);

  pRadioModel_->setFrequencies(llroundf(mhal_enb_config.uplink_frequency_hz_),
                               llroundf(mhal_enb_config.downlink_frequency_hz_));

  pRadioModel_->setNumResourceBlocks(mhal_enb_config.num_resource_blocks_);
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
                                              {"subid",                  {phyConfig.sSubId_}}
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
                                 {"otamanagerchannelenable", {platformConfig.sOtamManagerChannelEnable_}},
                                 {"otamanagergroup",         {platformConfig.sOtaManagerGroup_}},
                                 {"otamanagerloopback",      {platformConfig.sOtaManagerLoopback_}},
                                 {"otamanagerdevice",        {platformConfig.sOtaManagerDevice_}},
                                 {"eventservicegroup",       {platformConfig.sEventServiceGroup_}},
                                 {"eventservicedevice",      {platformConfig.sEventServiceDevice_}},
                                 {"controlportendpoint",     {platformConfig.sControlPortEndpoint_}}
                               });

  pNEMManager_->start();

  pNEMManager_->postStart();
}


void EMANELTE::MHAL::MHALENBImpl::start()
{
  logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, physicalCellId_=0x%x",
              __func__,
              physicalCellId_);

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
                                                      const RxData & rxData,
                                                      const PHY::OTAInfo & otaInfo,
                                                      const TxControlMessage & txControl)
{
  MHALCommon::handle_upstream_msg(data, rxData, otaInfo, txControl);
}


EMANE::SpectrumWindow EMANELTE::MHAL::MHALENBImpl::get_noise(FrequencyHz frequencyHz, 
                                                             const EMANE::Microseconds & span, 
                                                             const EMANE::TimePoint & sor)
{
  return pRadioModel_->getNoise(frequencyHz, span, sor);
}


long long unsigned int
EMANELTE::MHAL::MHALENBImpl::get_tx_prb_frequency(int prb_index)
{
  return pRadioModel_->getTxResourceBlockFrequency(prb_index);
}


void
EMANELTE::MHAL::MHALENBImpl::noise_processor(const uint32_t bin,
                                             const EMANE::Models::LTE::SpectrumWindowCache & spectrumWindowCache)
{
  // Track the combined transmit power of all UEs on shared channels
  EMANE::Models::LTE::SegmentSOTMap inBandSegmentPowerMap_mW;

  // Track the individual transmit power for segments from each source
  std::map<EMANE::NEMId, EMANE::Models::LTE::SegmentMap> nemRxPowers_dBm;

  // For the first pass, store the combined receive power for all transmitters sending on the same segment offset and duration
  for(auto & msg : pendingMessageBins_[bin].get())
    {
      auto rxControl = std::get<1>(msg);

      const auto & otaInfo = std::get<2>(msg);

      const auto & txControl = std::get<3>(msg);

      if(txControl.phy_cell_id() != physicalCellId_)
        {
          // transmitters on other cells are considered noise
          continue;
        }

      nemRxPowers_dBm.emplace(rxControl.nemId_, EMANE::Models::LTE::SegmentMap());

      EMANE::Models::LTE::SegmentMap & nemRxPowerMap_dBm(nemRxPowers_dBm.find(rxControl.nemId_)->second);

      for(auto & segment : otaInfo.segments_)
        {
          const auto rxPower_dBm = segment.getRxPowerdBm();
          const auto rxPower_mW  = EMANELTE::DB_TO_MW(segment.getRxPowerdBm());

          EMANE::Models::LTE::SegmentKey key{segment.getFrequencyHz(), segment.getOffset(), segment.getDuration()};

          nemRxPowerMap_dBm.emplace(key, rxPower_dBm);

          auto inBandIter = inBandSegmentPowerMap_mW.find(key);

          if(inBandIter == inBandSegmentPowerMap_mW.end())
            {
              inBandSegmentPowerMap_mW.emplace(key, EMANE::Models::LTE::SegmentSOTValue(otaInfo.sot_, otaInfo.sot_, rxPower_mW));
            }
          else
            {
              EMANE::TimePoint & minSot(std::get<0>(inBandIter->second));
              EMANE::TimePoint & maxSot(std::get<1>(inBandIter->second));
              float & partialRxPower_mW{std::get<2>(inBandIter->second)};

              minSot = std::min(minSot, otaInfo.sot_);
              maxSot = std::max(maxSot, otaInfo.sot_);
              partialRxPower_mW += rxPower_mW;
            }
        }
    }

  // For the second pass, determine the noise floor once all in band segment power is removed.
  // For dedicated channels there should only be 1 in-band transmitter assuming the enb scheduler does not
  // allocate uplink PDSCH resources to more than one UE (an assumption we are making currently).
  // For PUSCH and PRACH, we make an a simplifying assumption for now that inband receptions are time aligned and
  // orthogonal so do not interfere with each other; uplink sinr is calculated for each segment
  // as the segment receive power less the noisefloor of out of band contributions.
  EMANE::Models::LTE::SegmentMap outOfBandNoiseFloor_dBm;

  for(auto & segment : inBandSegmentPowerMap_mW)
    {
      const auto frequencyHz    = std::get<0>(segment.first);

      const auto spectrumWindow = spectrumWindowCache.find(frequencyHz);

      if(spectrumWindow == spectrumWindowCache.end())
        {
          logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, bin %u, no spectrumWindow cache info for freq %lu",
                      __func__,
                      bin,
                      frequencyHz);

          pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(frequencyHz);

          continue;
        }

      const auto & noiseData = std::get<0>(spectrumWindow->second);

      if(noiseData.empty())
        {
          logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, bin %u, freq %lu, no noise data",
                      __func__,
                      bin,
                      frequencyHz);

          pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(frequencyHz);

          continue;
        }

      const auto offset   = std::get<1>(segment.first);
      const auto duration = std::get<2>(segment.first);

      EMANE::TimePoint minSot, maxSot;
      double rxPower_mW;
      std::tie(minSot, maxSot, rxPower_mW) = segment.second;

      const auto rxPower_dBm = EMANELTE::MW_TO_DB(rxPower_mW);

      const auto minSor = minSot + offset;
      const auto maxEor = maxSot + offset + duration;

      // find the max out-of-band noise across the segment bins
      const auto rangeInfo = EMANE::Utils::maxBinNoiseFloorRange(spectrumWindow->second, rxPower_dBm, minSor, maxEor);

      const auto noiseFloor_dBm = rangeInfo.first;

      outOfBandNoiseFloor_dBm.emplace(EMANE::Models::LTE::SegmentKey(frequencyHz, offset, duration), noiseFloor_dBm);
    }

  // now process each message based on rx power from the source nem and
  // the out of band noise floor
  for(auto & msg : pendingMessageBins_[bin].get())
    {
      RxControl rxControl{};

      rxControl.rxData_ = std::move(std::get<1>(msg));

      const auto & otaInfo = std::get<2>(msg);

      const auto & txControl = std::get<3>(msg);

      // grab num segments here, some  stl list size() calls are not O(1)
      const size_t numSegments = otaInfo.segments_.size();

      if(txControl.phy_cell_id() != physicalCellId_)
        {
          // ignore transmitters from other cells
          continue;
        }

#ifdef ENABLE_INFO_2_LOGS
      logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, seqnum %lu, segments %zu",
                  __func__,
                  rxControl.rxData_.nemId_,
                  rxControl.rxData_.rx_seqnum_,
                  numSegments);
#endif
      double signalSum_mW = 0, noiseFloorSum_mW = 0;

      int segnum = -1;

      float peak_sum = 0.0;

      uint32_t num_samples = 0;

      EMANE::Models::LTE::SegmentMap segmentCache;

      // build segmentCache
      for(auto & segment : otaInfo.segments_)
        {
          const auto frequencyHz    = segment.getFrequencyHz();
          const auto spectrumWindow = spectrumWindowCache.find(frequencyHz);

          logger_.log(EMANE::DEBUG_LEVEL, "MHAL::PHY %s, src %hu, bin %u, freq %lu, offset %lu, duration %lu",
                      __func__,
                      rxControl.rxData_.nemId_,
                      bin,
                      frequencyHz,
                      segment.getOffset().count(),
                      segment.getDuration().count());

          ++segnum;

          if(spectrumWindow == spectrumWindowCache.end())
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, no spectrumWindow cache info for freq %lu",
                          __func__,
                          rxControl.rxData_.nemId_,
                          bin,
                          frequencyHz);

              pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(frequencyHz);

              continue;
            }

          float rxPower_dBm = segment.getRxPowerdBm();
          float rxPower_mW  = EMANELTE::DB_TO_MW(segment.getRxPowerdBm());

          double noiseFloor_dBm = outOfBandNoiseFloor_dBm.find(EMANE::Models::LTE::SegmentKey(segment.getFrequencyHz(), segment.getOffset(), segment.getDuration()))->second;
          double noiseFloor_mW = EMANELTE::DB_TO_MW(noiseFloor_dBm);

          signalSum_mW     += rxPower_mW;
          noiseFloorSum_mW += noiseFloor_mW;

          const auto sinr_dB = rxPower_dBm - noiseFloor_dBm;

          logger_.log(EMANE::DEBUG_LEVEL, "MHAL::PHY %s, src %hu, freq %lu, offset %lu, duration %lu, rxPower_dBm %0.1f, noisefloor_dbm %0.1f, sinr_dB %0.1f",
                      __func__,
                      rxControl.rxData_.nemId_,
                      segment.getFrequencyHz(),
                      segment.getOffset().count(),
                      segment.getDuration().count(),
                      rxPower_dBm,
                      noiseFloor_dBm,
                      sinr_dB);

          segmentCache.emplace(EMANE::Models::LTE::SegmentKey(frequencyHz, segment.getOffset(), segment.getDuration()), sinr_dB);

          pRadioModel_->getStatisticManager().updateRxFrequencyAvgNoiseFloor(frequencyHz, noiseFloor_mW);

#ifdef ENABLE_INFO_1_LOGS
          bool inBand = rangeInfo.second;
          const bool & bSignalInNoise{std::get<4>(spectrumWindow->second)};

          logger_.log(EMANE::INFO_LEVEL,
                      "MHAL::PHY %s, "
                      "src %hu, "
                      "sfIdx=%d, "
                      "seqnum %lu, "
                      "inband %d, "
                      "siginnoise %d, "
                      "freq %lu, "
                      "offs %lu, "
                      "dur %lu, "
                      "rxPwr %5.3f dBm, "
                      "nf %5.3lf dBm, "
                      "sinr %5.3lf dB",
                      __func__,
                      rxControl.rxData_.nemId_,
                      txControl.tti_tx(),
                      rxControl.rxData_.rx_seqnum_,
                      inBand,
                      bSignalInNoise,
                      frequencyHz,
                      segment.getOffset().count(),
                      segment.getDuration().count(),
                      rxPower_dBm,
                      noiseFloor_dBm,
                      sinr_dB);
#endif

          peak_sum += sinr_dB;

          num_samples++;
        } // end each segment

      // now check for number of pass/fail segments
      if(numSegments > 0)
        {
          const auto signalAvg_dBm = EMANELTE::MW_TO_DB(signalSum_mW / numSegments);

          const auto noiseFloorAvg_dBm = EMANELTE::MW_TO_DB(noiseFloorSum_mW / numSegments);

          UplinkSINRTesterImpl * pSINRTester = new UplinkSINRTesterImpl(signalAvg_dBm - noiseFloorAvg_dBm);

          rxControl.SINRTester_.setImpl(pSINRTester);

          rxControl.rxData_.peak_sum_ = peak_sum;

          rxControl.rxData_.num_samples_ = num_samples;

          if(txControl.uplink().has_prach())
            {
              putSINRResult(txControl.uplink().prach(),
                            rxControl,
                            pSINRTester,
                            pRadioModel_->noiseTestChannelMessage(txControl, txControl.uplink().prach(), segmentCache));
            }

          for(int i = 0; i < txControl.uplink().pucch_size(); ++i)
            {
              putSINRResult(txControl.uplink().pucch(i),
                            rxControl,
                            pSINRTester,
                            pRadioModel_->noiseTestChannelMessage(txControl, txControl.uplink().pucch(i), segmentCache));
            }

          for(int i = 0; i < txControl.uplink().pusch_size(); ++i)
            {
              putSINRResult(txControl.uplink().pusch(i),
                            rxControl,
                            pSINRTester,
                            pRadioModel_->noiseTestChannelMessage(txControl, txControl.uplink().pusch(i), segmentCache));
            }

          StatisticManager::ReceptionInfoMap receptionInfoMap;

          // add entry per src
          receptionInfoMap[rxControl.rxData_.nemId_] = StatisticManager::ReceptionInfoData{signalAvg_dBm,
                                                                                           noiseFloorAvg_dBm,
                                                                                           numSegments};
          statisticManager_.updateReceptionTable(receptionInfoMap);
        
          // lastly, make ready, take ownership of data and control
          readyMessageBins_[bin].get().push_back(RxMessage{std::move(std::get<0>(msg)), std::move(rxControl)});
        }
    } // end for each msg

}


void
EMANELTE::MHAL::MHALENBImpl::putSINRResult(const ChannelMessage & channel_message,
                                           RxControl & rxControl,
                                           UplinkSINRTesterImpl * pSINRTester,
                                           bool received)
{
  CHANNEL_TYPE ctype = channel_message.channel_type();

  if(channel_message.has_rnti())
    {
      logger_.log(EMANE::DEBUG_LEVEL, "MHAL::PHY %s insert sinr result, src %hu, seqnum %lu, chantype %d, rnti %hu",
                  __func__,
                  rxControl.rxData_.nemId_,
                  rxControl.rxData_.rx_seqnum_,
                  ctype,
                  channel_message.rnti());

      pSINRTester->rntiChannelSINRResults_.emplace(ChannelRNTI(ctype, channel_message.rnti()), received);
    }
  else
    {
      logger_.log(EMANE::DEBUG_LEVEL, "MHAL::PHY %s insert sinr result, src %hu, seqnum %lu, chantype %d",
                  __func__,
                  rxControl.rxData_.nemId_,
                  rxControl.rxData_.rx_seqnum_,
                  ctype);

      pSINRTester->channelSINRResults_.emplace(ctype, received);
    }
}


bool
EMANELTE::MHAL::MHALENBImpl::get_messages(RxMessages & messages, timeval & tv_sor)
{
  bool in_step = false;

  timing_.lockTime();

  timeval tv_now, tv_diff, tv_process_diff, tv_sf_time = timing_.getCurrSfTime();

  gettimeofday(&tv_now, NULL);

  // get the process time for the calling thread for the time remaining in this subframe
  timersub(&tv_now, &tv_sf_time, &tv_process_diff);

  // get the time till the next subframe
  timersub(&timing_.getNextSfTime(), &tv_now, &tv_diff);

  // this is where we set the pace for the system pulse
  if(timercmp(&tv_diff, &tv_zero_, >))
    {
      // next sf is still in the future
      // wait until next subframe time
      select(0, NULL, NULL, NULL, &tv_diff);

      in_step = true;
    }
  else
    {
      const time_t dT = timing_.ts_sf_interval_usec() + tvToUseconds(tv_diff);

      if(dT < 0)
        {
          // we passed the next sf mark, continue and try to catch up on subsequent call(s)
          in_step = false;
        }
      else
        {
          // we passed the next sf mark, but are still within the sf window
          in_step = true;
        }
    }

  // use sf_time for the bin
  const uint32_t bin = getMessageBin(tv_sf_time, timing_.ts_sf_interval_usec());

  pendingMessageBins_[bin].lockBin();

  // now advance the subframe times, curr -> next, next -> next+1
  uint32_t nextbin{timing_.stepTime()};

  pendingMessageBins_[nextbin].lockBin();

  // check for orphans
  if(size_t orphaned = pendingMessageBins_[nextbin].clearAndCheck())
    {
      logger_.log(EMANE::ERROR_LEVEL, "MHAL::RADIO %s curr_sf %ld:%06ld, bin %u, purge %zu pending orphans",
                  __func__,
                  timing_.getCurrSfTime().tv_sec,
                  timing_.getCurrSfTime().tv_usec,
                  nextbin,
                  orphaned);

      statisticManager_.updateOrphanedMessages(nextbin, orphaned);
    }

  clearReadyMessages(nextbin);

  pendingMessageBins_[nextbin].unlockBin();

  // get msgs from the previous subframe
  noise_worker(bin, tv_sf_time);

  // set the sor to the sf time (time aligned via lte time advance)
  tv_sor = tv_sf_time;

  timing_.unlockTime();

#ifdef ENABLE_INFO_1_LOGS
  const timeval tv_curr_sf = timing_.getCurrSfTime();
  logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s bin %u, sor %ld:%06ld, prev_sf %ld:%06ld, curr_sf %ld:%06ld, %zu msgs ready",
              __func__,
              bin,
              tv_sor.tv_sec,
              tv_sor.tv_usec,
              tv_sf_time.tv_sec,
              tv_sf_time.tv_usec,
              tv_curr_sf.tv_sec,
              tv_curr_sf.tv_usec,
              pendingMessageBins_[bin].getReady().size());
#endif

  // transfer to caller
  messages = std::move(readyMessageBins_[bin].get());

  // clear bin
  pendingMessageBins_[bin].clear();

  readyMessageBins_[bin].clear();

  pendingMessageBins_[bin].unlockBin();

  statisticManager_.updateHandoffMessages(bin, messages.size());

  statisticManager_.tallySubframeProcessTime(bin, tv_process_diff, !in_step);

  return in_step;
}
