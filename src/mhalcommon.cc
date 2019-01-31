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


#include "mhalcommon.h"
#include "configmanager.h"


#undef ENABLE_INFO_1_LOGS
#undef ENABLE_INFO_2_LOGS

// uncomment to enable info 1/2 logs
//#define ENABLE_INFO_1_LOGS
//#define ENABLE_INFO_2_LOGS



void
EMANELTE::MHAL::MHALCommon::initialize(uint32_t sf_interval_msec, const mhal_config_t & mhal_config)
{
  set_logger(mhal_config);

  // subframe step is 1 msec/subframe
  timing_.set_tv_sf_interval(0, sf_interval_msec * 1000);

  timing_.set_tv_slot_interval(0, sf_interval_msec * 1000/2);

  timing_.set_ts_sf_interval_usec(sf_interval_msec * 1000);

  int parent_policy = 0;
  struct sched_param parent_priority = {0};

  // get caller priority
  pthread_getschedparam(pthread_self(), &parent_policy, &parent_priority);

  // elevate so emane can inherit
  if(parent_policy == SCHED_OTHER)
    {
      set_thread_priority(pthread_self(), SCHED_RR, 50);
    }
  else
    {
      set_thread_priority(pthread_self(), parent_policy, parent_priority.sched_priority + 1);
    }

  statisticManager_.start(mhal_config.statistic_service_endpoint, sf_interval_msec);

  init_emane();

  // reset caller priority
  set_thread_priority(pthread_self(), parent_policy, parent_priority.sched_priority);
}


void
EMANELTE::MHAL::MHALCommon::start(uint32_t nof_advance_sf)
{
  if(! state_.isStarted())
    {
      for(size_t bin = 0; bin < EMANELTE::NUM_SF_PER_FRAME; ++bin)
        {
          pendingMessageBins_[bin].lockBin();
          pendingMessageBins_[bin].clear();
          clearReadyMessages(bin);
          pendingMessageBins_[bin].unlockBin();
        }

      timing_.lockTime();

      timing_.alignTime(nof_advance_sf);

      const auto & tv_curr_sf = timing_.getCurrSfTime();

      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s curr_sf_time %ld:%06ld", 
                      __func__, tv_curr_sf.tv_sec, tv_curr_sf.tv_usec);

      timing_.unlockTime();

      state_.start();
    }
  else
    {
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s already running", __func__);
    }
}


void
EMANELTE::MHAL::MHALCommon::set_tti(uint16_t curr_tti)
{
  logger_.log(EMANE::DEBUG_LEVEL, " MHAL::RADIO %s%u", __func__, curr_tti);
}


void
EMANELTE::MHAL::MHALCommon::set_logger(const mhal_config_t & mhal_config)
{
  auto & configManager = ConfigManager::getInstance();

  auto & platformConfig = configManager.getPlatformConfig();

  logger_.setLogLevel(EMANE::LogLevel::DEBUG_LEVEL);

  configManager.setLogger(logger_);

  configManager.configure(mhal_config.emane_configfile);

  if(platformConfig.sLogLevel_ >= "4")
    {
      logger_.setLogLevel(EMANE::LogLevel::DEBUG_LEVEL);
    }
  else if(platformConfig.sLogLevel_ == "3")
    {
      logger_.setLogLevel(EMANE::LogLevel::INFO_LEVEL);
    }
  else if(platformConfig.sLogLevel_ == "2")
    {
      logger_.setLogLevel(EMANE::LogLevel::ERROR_LEVEL);
    }
  else if(platformConfig.sLogLevel_ == "1")
    {
      logger_.setLogLevel(EMANE::LogLevel::ABORT_LEVEL);
    }
  else
    {
      logger_.setLogLevel(EMANE::LogLevel::NOLOG_LEVEL);
    }

  if(!platformConfig.sLogFileName_.empty())
    {
      logger_.redirectLogsToFile(platformConfig.sLogFileName_.c_str());
    }
}

  
void
EMANELTE::MHAL::MHALCommon::send_msg(const Data & data,
                                     TxControlMessage & txControl)
{
  txControl.set_subframe_duration_microsecs(timing_.ts_sf_interval_usec());

  timeval tv_sf_time{txControl.sf_time().ts_sec(), txControl.sf_time().ts_usec()};

  if(TX_TIME_ADJUST > 0)
    {
      // pull back tx timestamp to allow for greater processing time on at the receiver
      timeval tv_adjust = tsToTv(timing_.ts_sf_interval_usec() * TX_TIME_ADJUST);

      timersub(&tv_sf_time, &tv_adjust, &tv_sf_time);
    }
 
#ifdef ENABLE_INFO_1_LOGS                   
  const timeval & tv_curr_sf = timing_.getCurrSfTime();
  logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s seqnum %lu, tti_tx %u, curr_sf %ld:%06ld, adjust %d, sf_time %ld:%06ld",
              __func__,
              txControl.tx_seqnum(),
              txControl.tti_tx(),
              tv_curr_sf.tv_sec,
              tv_curr_sf.tv_usec,
              TX_TIME_ADJUST,
              tv_sf_time.tv_sec,
              tv_sf_time.tv_usec);
#endif

  send_downstream(data, txControl, EMANE::TimePoint(EMANE::Microseconds(tvToUseconds(tv_sf_time))));
}


void EMANELTE::MHAL::MHALCommon::stop()
{
  logger_.log(EMANE::INFO_LEVEL, " MHAL::RADIO %s", __func__);

  state_.stop();

  statisticManager_.stop();
}


void
EMANELTE::MHAL::MHALCommon::set_thread_priority(pthread_t tid, int policy, int priority)
{
  struct sched_param param = {priority};

  if(pthread_setschedparam(tid, policy, &param) < 0)
    {
      logger_.log(EMANE::ERROR_LEVEL, "could not set thread %ld, priority/policy %d:%d, %s\n", 
                  tid, priority, policy, strerror(errno));
    }
  else
    {
      logger_.log(EMANE::INFO_LEVEL, "set thread %ld, priority/policy %d:%d\n", 
                  tid, priority, policy);
    }
}



void EMANELTE::MHAL::MHALCommon::noise_worker(const uint32_t bin, const timeval & tv_sf_start __attribute__((unused)))
{
  struct timeval tv_in, tv_out, tv_diff;

  // track work time
  gettimeofday(&tv_in, NULL);

#ifdef ENABLE_INFO_2_LOGS                   
  logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, sf_start %ld:%06ld, bin %u, %zu pending", 
                  __func__,
                  tv_sf_start.tv_sec, 
                  tv_sf_start.tv_usec,
                  bin,
                  pendingMessageBins_[bin].getPending().size());
#endif

  clearReadyMessages(bin);

  // container for all freqs and energy for this subframe
  // allows for consulting the spectrum sevice once and only once for each freq/timerange
  EMANE::Models::LTE::SpectrumWindowCache spectrumWindowCache;

  // load the spectrumWindow cache for each frequency
  for(auto & span : pendingMessageBins_[bin].getSegmentSpans())
    {
      const auto & minSor      = std::get<0>(span.second);
      const auto & maxEor      = std::get<1>(span.second);
      const auto & frequencyHz = span.first;
      const auto duration      = std::chrono::duration_cast<EMANE::Microseconds>(maxEor.time_since_epoch() - minSor.time_since_epoch());

#ifdef ENABLE_INFO_1_LOGS                   
      auto & numEntries  = std::get<2>(span.second);
      logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s, freq %lu, minSor %f, maxEor %f, duration %ld, numEntries %zu", 
                      __func__,
                      frequencyHz,
                      minSor.time_since_epoch().count()/1e9,
                      maxEor.time_since_epoch().count()/1e9,
                      duration.count(),
                      numEntries);
#endif

      spectrumWindowCache[frequencyHz] = get_noise(frequencyHz, duration, minSor);
    }

  statisticManager_.updateDequeuedMessages(bin, pendingMessageBins_[bin].get().size());

  noise_processor(bin, spectrumWindowCache);

  // done with pending msgs
  pendingMessageBins_[bin].clear();

  gettimeofday(&tv_out, NULL);

  timersub(&tv_out, &tv_in, &tv_diff);

  // update process time
  statisticManager_.updateNoiseProcessDelay(bin, tvToSeconds(tv_diff));
}


void
EMANELTE::MHAL::MHALCommon::handle_upstream_msg(const Data & data,
                                                const RxData & rxData,
                                                const PHY::OTAInfo & otaInfo,
                                                const TxControlMessage & txControl)
{
  if(state_.isRunning())
    {
      const auto sor = otaInfo.sot_; // LTE timing advance negates propagation delay (sor == sot)
      const auto eor = sor + otaInfo.span_;
      const auto now = EMANE::Clock::now();
      const auto dT  = std::chrono::duration_cast<EMANE::Microseconds>(eor.time_since_epoch() - now.time_since_epoch());

      // get bin for msg sf_time (tti) this is the sf that the msg is expected to arrive tx/rx
      const uint32_t bin = getMessageBin(rxData.sf_time_, timing_.ts_sf_interval_usec());

      timeval tv_diff;

      // get ota diff
      timersub(&rxData.rx_time_, &rxData.tx_time_, &tv_diff);

      statisticManager_.updateRxPacketOtaDelay(bin, tvToSeconds(tv_diff));

      // eor should be in the future
      if(dT > EMANE::Microseconds{0})
        {
          statisticManager_.updateEnqueuedMessages(bin);

          // lock this bin
          pendingMessageBins_[bin].lockBin();

          // add to the pending queue for this bin
          pendingMessageBins_[bin].add(rxData.sf_time_,
                                       bin,
                                       PendingMessage{data, rxData, otaInfo, txControl},
                                       statisticManager_);

          // unlock bin
          pendingMessageBins_[bin].unlockBin();
        }
      else
        {
          // rx late too late, discard
          statisticManager_.updateLateMessages(bin);
        }

#ifdef ENABLE_INFO_1_LOGS
      const auto & tv_curr_sf = timing_.getCurrSfTime();
      logger_.log(EMANE::INFO_LEVEL, "MHAL::PHY %s seqnum %lu, bin %u, curr_sf %ld:%06ld, sf_time %ld:%06ld, sot %f, span %ld, sor %f, eor %f, dT %ld usec, pending %zu", 
                      __func__,
                      rxData.rx_seqnum_,
                      bin,
                      tv_curr_sf.tv_sec,
                      tv_curr_sf.tv_usec,
                      rxData.sf_time_.tv_sec,
                      rxData.sf_time_.tv_usec,
                      otaInfo.sot_.time_since_epoch().count()/1e9,
                      otaInfo.span_.count(),
                      sor.time_since_epoch().count()/1e9,
                      eor.time_since_epoch().count()/1e9,
                      dT.count(),
                      pendingMessageBins_[bin].getPending().size());
#endif
    }
  else
    {
      logger_.log(EMANE::ERROR_LEVEL, " MHAL::PHY %s not started, discard", __func__);
    }
}


void
EMANELTE::MHAL::MHALCommon::clearReadyMessages(const uint32_t bin)
{
  if(size_t orphaned = readyMessageBins_[bin].clearAndCheck())
    {
      logger_.log(EMANE::ERROR_LEVEL, "MHAL::RADIO %s, bin %u, purge %zu ready orphans",
                  __func__,
                  bin,
                  orphaned);

      statisticManager_.updateOrphanedMessages(bin, orphaned);
    }
}
