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

namespace {
   const auto ZERO_USEC = EMANE::Microseconds{0};
}

void
EMANELTE::MHAL::MHALCommon::initialize(uint32_t sf_interval_msec, const mhal_config_t & mhal_config)
{
  set_logger(mhal_config);

  // subframe step is 1 msec/subframe
  timing_.set_tv_sf_interval(0, sf_interval_msec * 1000);

  timing_.set_tv_slot_interval(0, sf_interval_msec * 1000/2);

  timing_.set_ts_sf_interval_usec(sf_interval_msec * 1000);

  nof_advance_sf_ = 0;

  int parent_policy = 0;
  struct sched_param parent_priority = {0};

  // get caller priority
  pthread_getschedparam(pthread_self(), &parent_policy, &parent_priority);

  // elevate so emane child can inherit
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
  // save no of advance frames for later,
  // we will not go into start state until the stack
  // actually begins the rx threads
  nof_advance_sf_ = nof_advance_sf;
}


void
EMANELTE::MHAL::MHALCommon::start_rx_i()
{
  if(! state_.isStarted())
    {
      for(size_t bin = 0; bin < EMANELTE::NUM_SF_PER_FRAME; ++bin)
        {
          pendingMessageBins_[bin].lockBin();
          pendingMessageBins_[bin].clear();
          clearReadyMessages_safe(bin);
          pendingMessageBins_[bin].unlockBin();
        }

      // timing info must be locked by caller
      timing_.alignTime(nof_advance_sf_);

      const auto & tv_curr_sf = timing_.getCurrSfTime();

      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s curr_sf_time %ld:%06ld", 
                  __func__, tv_curr_sf.tv_sec, tv_curr_sf.tv_usec);

      state_.start();
    }
}


void
EMANELTE::MHAL::MHALCommon::set_tti(uint16_t curr_tti __attribute__((unused)))
{
  // not used
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



void EMANELTE::MHAL::MHALCommon::noiseWorker_safe(const uint32_t bin, const timeval & tv_sf_start __attribute__((unused)))
{
  struct timeval tv_in, tv_out, tv_in_out_diff;

  // track work time
  gettimeofday(&tv_in, NULL);

  const size_t numMessages = pendingMessageBins_[bin].get().size();

  clearReadyMessages_safe(bin);

  // container for all freqs and energy for this subframe
  // allows for consulting the spectrum sevice once and only once for each freq/timerange
  EMANE::Models::LTE::SpectrumWindowCache spectrumWindowCache;

  // load the spectrumWindow cache for each frequency in this msg
  for(auto & segmentSpan : pendingMessageBins_[bin].getSegmentSpans())
    {
      const auto & minSor      = SegmentTimeSpan_sor(segmentSpan.second);
      const auto & maxEor      = SegmentTimeSpan_eor(segmentSpan.second);
      const auto & frequencyHz = segmentSpan.first;
      const auto duration      = std::chrono::duration_cast<EMANE::Microseconds>(maxEor - minSor);

#ifdef ENABLE_INFO_2_LOGS                   
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, bin %u, freq %lu%s, minSor %f, maxEor %f, duration %ld, numEntries %zu", 
                  __func__,
                  bin,
                  frequencyHz,
                  spectrumWindowCache.count(frequencyHz) != 0 ? " not-unique" : "",
                  minSor.time_since_epoch().count()/1e9,
                  maxEor.time_since_epoch().count()/1e9,
                  duration.count(),
                  SegmentTimeSpan_num(segmentSpan.second));
#endif

      spectrumWindowCache[frequencyHz] = get_noise(frequencyHz, duration, minSor);
    }

  statisticManager_.updateDequeuedMessages(bin, numMessages);

  noise_processor(bin, spectrumWindowCache);

  // done with pending msgs
  pendingMessageBins_[bin].clear();

  gettimeofday(&tv_out, NULL);

  timersub(&tv_out, &tv_in, &tv_in_out_diff);

  // update process time
  statisticManager_.updateNoiseProcessDelay(bin, tvToSeconds(tv_in_out_diff));
}


void
EMANELTE::MHAL::MHALCommon::handle_upstream_msg(const Data & data,
                                                const RxControl & rxControl,
                                                const PHY::OTAInfo & otaInfo,
                                                const TxControlMessage & txControl)
{
  const auto tp_now = EMANE::Clock::now(); 

  if(state_.isRunning())
    {
      const auto sor = otaInfo.sot_; // LTE timing advance negates propagation delay (sor == sot)
      const auto eor = sor + otaInfo.span_;

      // should be ~4 sf in the future
      const auto dt = std::chrono::duration_cast<EMANE::Microseconds>(eor.time_since_epoch() - tp_now.time_since_epoch());

      // get bin for msg sf_time (tti) this is the sf that the msg is expected to arrive tx/rx
      const uint32_t bin = getMessageBin(rxControl.sf_time_, timing_.ts_sf_interval_usec());

      timeval tv_ota_diff;

      // get ota diff
      timersub(&rxControl.rx_time_, &rxControl.tx_time_, &tv_ota_diff);

      statisticManager_.updateRxPacketOtaDelay(bin, tvToSeconds(tv_ota_diff));

      // eor should be in the future
      if(dt >= ZERO_USEC)
        {
          statisticManager_.updateEnqueuedMessages(bin);

          // lock this bin
          pendingMessageBins_[bin].lockBin();

          // add to the pending queue for this bin
          pendingMessageBins_[bin].add(rxControl.sf_time_,
                                       bin,
                                       PendingMessage{data, rxControl, otaInfo, txControl},
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
          logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s seqnum %lu, bin %u, curr_time %f, sot %f, span %ld, sor %f, eor %f, dt %ld usec, msgs %zu", 
                      __func__,
                      rxControl.rx_seqnum_,
                      bin,
                      tp_now.time_since_epoch().count()/1e9,
                      otaInfo.sot_.time_since_epoch().count()/1e9,
                      otaInfo.span_.count(),
                      sor.time_since_epoch().count()/1e9,
                      eor.time_since_epoch().count()/1e9,
                      dt.count(),
                      pendingMessageBins_[bin].get().size());
#endif
    }
  else
    {
      logger_.log(EMANE::ERROR_LEVEL, " MHAL::RADIO %s not started, discard", __func__);
    }
}


bool
EMANELTE::MHAL::MHALCommon::get_messages(RxMessages & messages, timeval & r_tv_sor)
{
  timing_.lockTime();

  start_rx_i();

  const timeval tv_curr_sf = timing_.getCurrSfTime();

  const timeval tv_next_sf_time = timing_.getNextSfTime();

  timeval tv_now, tv_delay;

  gettimeofday(&tv_now, NULL);

  // get the delta to the begin next subframe (end of this sub frame)
  timersub(&tv_next_sf_time, &tv_now, &tv_delay);

  const time_t time_to_wait_usec = tvToUseconds(tv_delay);

  bool bSfTimeInStep = true;

  // this is where we set the pace for the system pulse
  if(time_to_wait_usec > 0)
    {
      // end of this subframe time is in the future
      select(0, NULL, NULL, NULL, &tv_delay);
    }
  // need to play catchup, run w/o delay
  else
    {
      bSfTimeInStep = false;
#ifdef ENABLE_INFO_1_LOGS
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s curr_sf_time %ld:%06ld, off by %ld usec",
                  __func__,
                  tv_curr_sf.tv_sec,
                  tv_curr_sf.tv_usec,
                  time_to_wait_usec);
#endif
    }

  // use sf_time for the bin
  const uint32_t bin = getMessageBin(tv_curr_sf, timing_.ts_sf_interval_usec());

  // now advance the subframe times, curr -> next, next -> next+1
  const uint32_t nextbin{timing_.stepTime()};

  // clear next subrame stale data
  pendingMessageBins_[nextbin].lockBin();

  clearPendingMessages_safe(nextbin);

  clearReadyMessages_safe(nextbin);

  pendingMessageBins_[nextbin].unlockBin();

  // process this subframe bin now
  pendingMessageBins_[bin].lockBin();

  noiseWorker_safe(bin, tv_curr_sf);

  // set the sor to the subframe time (time aligned via lte time advance)
  r_tv_sor = tv_curr_sf;

  timing_.unlockTime();

  const auto offset_usec = timing_.ts_sf_interval_usec() - time_to_wait_usec;

  // dont bother with stale data
  if(not (offset_usec > timing_.ts_sf_interval_usec()))
   {
     // transfer to caller, all done with bin data
     messages = std::move(readyMessageBins_[bin].get());
   }

#ifdef ENABLE_INFO_1_LOGS                   
     logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s bin %u, sor %ld:%06ld, curr_sf %ld:%06ld, delay %ld usec, %zu msgs ready",
                 __func__,
                 bin,
                 r_tv_sor.tv_sec,
                 r_tv_sor.tv_usec,
                 tv_curr_sf.tv_sec,
                 tv_curr_sf.tv_usec,
                 time_to_wait_usec,
                 messages.size());
#endif

  // clear bin for this subframe
  pendingMessageBins_[bin].clear();

  readyMessageBins_[bin].clear();

  pendingMessageBins_[bin].unlockBin();

  statisticManager_.updateHandoffMessages(bin, messages.size());

  // get the process time for the calling thread for the time remaining in this subframe
  timeval tv_curr_sf_remain;

  timersub(&tv_now, &tv_curr_sf, &tv_curr_sf_remain);

  statisticManager_.tallySubframeProcessTime(bin, tv_curr_sf_remain);

  return bSfTimeInStep;
}

void
EMANELTE::MHAL::MHALCommon::clearReadyMessages_safe(const uint32_t bin)
{
  if(auto numOrphans = readyMessageBins_[bin].clearAndCheck())
    {
#ifdef ENABLE_INFO_1_LOGS                   
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, bin %u, purge %zu ready orphans",
                  __func__,
                  bin,
                  numOrphans);
#endif

      statisticManager_.updateOrphanedMessages(bin, numOrphans);
    }
}

void
EMANELTE::MHAL::MHALCommon::clearPendingMessages_safe(const uint32_t bin)
{
  if(auto numOrphans = pendingMessageBins_[bin].clearAndCheck())
    {
#ifdef ENABLE_INFO_1_LOGS                   
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, bin %u, purge %zu pending orphans",
                  __func__,
                  bin,
                  numOrphans);
#endif

      statisticManager_.updateOrphanedMessages(bin, numOrphans);
    }
}
