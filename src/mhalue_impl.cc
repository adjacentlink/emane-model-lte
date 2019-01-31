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
                                                     const RxData & rxData,
                                                     const PHY::OTAInfo & otaInfo,
                                                     const TxControlMessage & txControl)
{
  // ue starts with cell search, until then receiver is off
  if(!state_.ue_init())
    {
      logger_.log(EMANE::INFO_LEVEL, " MHAL::PHY %s ue not in cell search yet, discard", __func__);

      return;
    }

  MHALCommon::handle_upstream_msg(data, rxData, otaInfo, txControl);
}


EMANE::SpectrumWindow EMANELTE::MHAL::MHALUEImpl::get_noise(FrequencyHz frequencyHz, 
                                                            const EMANE::Microseconds & span, 
                                                            const EMANE::TimePoint & sor)
{
  return pRadioModel_->getNoise(frequencyHz, span, sor);
}


void
EMANELTE::MHAL::MHALUEImpl::begin_cell_search()
{
  logger_.log(EMANE::INFO_LEVEL, "MHAL %s", __func__);

  // on cell search, configure for the largest bandwidth to
  // ensure packet reception from any enb (register all possible FOI
  // for the configured receive frequency)
  pRadioModel_->setNumResourceBlocks(100, true);

  for(size_t bin = 0; bin < EMANELTE::NUM_SF_PER_FRAME; ++bin)
    {
      pendingMessageBins_[bin].lockBin();
      pendingMessageBins_[bin].clear();
      readyMessageBins_[bin].clear();
      pendingMessageBins_[bin].unlockBin();
    }

  timing_.lockTime();

  timing_.alignTime(0);
  state_.ue_init(true);

  timing_.unlockTime();
}


void
EMANELTE::MHAL::MHALUEImpl::set_frequencies(float ul_freq, float dl_freq)
{
  pRadioModel_->setFrequencies(llroundf(dl_freq),         // rx freq
                               llroundf(ul_freq));        // tx freq

  pRadioModel_->setNumResourceBlocks(100, true);
}


void
EMANELTE::MHAL::MHALUEImpl::set_num_resource_blocks(int num_resource_blocks)
{
  pRadioModel_->setNumResourceBlocks(num_resource_blocks);
}


long long unsigned int
EMANELTE::MHAL::MHALUEImpl::get_tx_prb_frequency(int prb_index)
{
  return pRadioModel_->getTxResourceBlockFrequency(prb_index);
}


void
EMANELTE::MHAL::MHALUEImpl::noise_processor(const uint32_t bin,
                                            const EMANE::Models::LTE::SpectrumWindowCache & spectrumWindowCache)
{
  // for each pending message
  for(auto & msg : pendingMessageBins_[bin].get())
    {
      RxControl rxControl;

      // get the rx control info
      rxControl.rxData_ = std::move(std::get<1>(msg));

      // get the ota info
      const auto & otaInfo = std::get<2>(msg);

      // get the TxControl message
      const auto & txControl = std::get<3>(msg);

      // reception stats
      StatisticManager::ReceptionInfoMap receptionInfoMap;

#ifdef ENABLE_INFO_2_LOGS
      logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, src %hu, seqnum %lu, segments %zu",
                  __func__,
                  rxControl.rxData_.nemId_,
                  rxControl.rxData_.rx_seqnum_,
                  otaInfo.segments_.size());
#endif
      double signalSum_mW = 0, noiseFloorSum_mW = 0;

      int segnum = -1;

      double peak_sum = 0.0;

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

          const auto & noiseData = std::get<0>(spectrumWindow->second);

          if(noiseData.empty())
            {
              logger_.log(EMANE::ERROR_LEVEL, "MHAL::PHY %s, src %hu, bin %u, freq %lu, no noise data",
                          __func__,
                          rxControl.rxData_.nemId_,
                          bin,
                          frequencyHz);

              pRadioModel_->getStatisticManager().updateRxFrequencySpectrumError(segment.getFrequencyHz());

              continue;
            }

          double rxPower_dBm = segment.getRxPowerdBm();
          double rxPower_mW  = EMANELTE::DB_TO_MW(segment.getRxPowerdBm());

          const auto segmentSor = otaInfo.sot_ + segment.getOffset();
          const auto segmentEor = segmentSor   + segment.getDuration();
          const auto rangeInfo = EMANE::Utils::maxBinNoiseFloorRange(spectrumWindow->second, rxPower_dBm, segmentSor, segmentEor);

          double noiseFloor_dBm = rangeInfo.first;
          double noiseFloor_mW = EMANELTE::DB_TO_MW(noiseFloor_dBm);

          signalSum_mW     += rxPower_mW;
          noiseFloorSum_mW += noiseFloor_mW;

          double sinr_dB = rxPower_dBm - noiseFloor_dBm;

          segmentCache.insert(std::pair<EMANE::Models::LTE::SegmentKey, float>(
             EMANE::Models::LTE::SegmentKey(frequencyHz, segment.getOffset(), segment.getDuration()), sinr_dB));

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
      if(otaInfo.segments_.size() > 0)
        {
          rxControl.rxData_.peak_sum_ = peak_sum;

          rxControl.rxData_.num_samples_ = num_samples;

          bool pcfichPass = pRadioModel_->noiseTestChannelMessage(txControl, txControl.downlink().pcfich(), segmentCache);

          bool pbchPass{false};

          if(txControl.downlink().has_pbch())
            {
              pbchPass = pRadioModel_->noiseTestChannelMessage(txControl, txControl.downlink().pbch(), segmentCache);
            }

          DownlinkSINRTesterImpl * pSINRTester = \
            new DownlinkSINRTesterImpl(pRadioModel_, std::move(txControl), std::move(segmentCache), pcfichPass, pbchPass);

          rxControl.SINRTester_.setImpl(pSINRTester);

          // make ready, take ownership of data and control
          readyMessageBins_[bin].get().push_back(RxMessage{std::move(std::get<0>(msg)), std::move(rxControl)});

          const auto signalAvg_dBm = EMANELTE::MW_TO_DB(signalSum_mW / otaInfo.segments_.size());

          const auto noiseFloorAvg_dBm = EMANELTE::MW_TO_DB(noiseFloorSum_mW / otaInfo.segments_.size());

          receptionInfoMap[rxControl.rxData_.nemId_] = StatisticManager::ReceptionInfoData{signalAvg_dBm,
                                                                                           noiseFloorAvg_dBm,
                                                                                           otaInfo.segments_.size()};
        }

      statisticManager_.updateReceptionTable(receptionInfoMap);
    } // end for each msg
}


bool
EMANELTE::MHAL::MHALUEImpl::get_messages(RxMessages & messages, timeval & tv_sor)
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
