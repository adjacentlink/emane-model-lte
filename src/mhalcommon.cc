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
#include <math.h>

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
  if(! state_.isStarted())
   {
     clearBins_i();

     timing_.lockTime();

     timing_.alignTime(nof_advance_sf);

     logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s curr_sf_time %ld:%06ld, nof_advance %u", 
                  __func__, timing_.getCurrSfTime().tv_sec, timing_.getCurrSfTime().tv_usec, nof_advance_sf);

     timing_.unlockTime();

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
  txControl.set_subframe_duration_microsecs(timing_.tsSfIntervalUsec());

  timeval tvSfTime{txControl.sf_time().ts_sec(), txControl.sf_time().ts_usec()};

  if(TX_TIME_ADJUST > 0)
    {
      // pull back tx timestamp to allow for greater processing time on at the receiver
      timeval tvAdjust = tsToTv(timing_.tsSfIntervalUsec() * TX_TIME_ADJUST);

      timersub(&tvSfTime, &tvAdjust, &tvSfTime);
    }
 
  send_downstream(data, txControl, EMANE::TimePoint(EMANE::Microseconds(tvToUseconds(tvSfTime))));
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



void EMANELTE::MHAL::MHALCommon::noiseWorker_safe(const uint32_t bin)
{
  struct timeval tvIn, tvOut, tvDiff;

  // track work time start
  gettimeofday(&tvIn, NULL);

  const size_t numMessages = pendingMessageBins_[bin].get().size();

  clearReadyMessages_safe(bin);

  // container for all freqs and energy for this subframe per rxAntenna
  // allows for consulting the spectrum sevice once and only once for each freq/span
  EMANE::Models::LTE::AntennaSpectrumWindowCache antennaSpectrumWindowCache;

  // load the spectrumWindow cache for each frequency in this msg
  for(auto & segmentSpans : pendingMessageBins_[bin].getAntennaSegmentSpans())
   {
     const auto rxAntennaId = segmentSpans.first;

     for(auto & segmentSpan : segmentSpans.second)
      {
        const auto & minSor      = SegmentTimeSpan_Sor_Get(segmentSpan.second);
        const auto & maxEor      = SegmentTimeSpan_Eor_Get(segmentSpan.second);
        const auto & frequencyHz = segmentSpan.first;
        const auto duration      = std::chrono::duration_cast<EMANE::Microseconds>(maxEor - minSor);

        // per rx antenna
        antennaSpectrumWindowCache[rxAntennaId][frequencyHz] = get_noise(rxAntennaId, frequencyHz, duration, minSor);
      }
   }

  statisticManager_.updateDequeuedMessages(bin, numMessages);

  // load up the ready messages
  noise_processor(bin, antennaSpectrumWindowCache);

  // done with pending msgs
  pendingMessageBins_[bin].clear();

  gettimeofday(&tvOut, NULL);

  timersub(&tvOut, &tvIn, &tvDiff);

  // update process time
  statisticManager_.updateNoiseProcessDelay(bin, tvToSeconds(tvDiff));
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
      const auto & sor = otaInfo.sot_; // LTE timing advance negates propagation delay (sor == sot)

      // should be ~4 sf in the future
      const auto dt = std::chrono::duration_cast<EMANE::Microseconds>(sor.time_since_epoch() - tp_now.time_since_epoch());

      // get bin for msg sf_time (tti) this is the sf that the msg is expected to arrive tx/rx
      const uint32_t bin = getMessageBin(rxControl.sf_time_, timing_.tsSfIntervalUsec());

      timeval tvDiff;

      // get ota diff
      timersub(&rxControl.rx_time_, &rxControl.tx_time_, &tvDiff);

      statisticManager_.updateRxPacketOtaDelay(bin, tvToSeconds(tvDiff));

      // sor should be now or in the future
      if(dt >= EMANE::Microseconds::zero())
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

          logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s discard seqnum %lu, bin %u, curr_time %f, sot %f, dt %ld usec", 
                      __func__,
                      rxControl.rx_seqnum_,
                      bin,
                      tp_now.time_since_epoch().count()/1e9,
                      sor.time_since_epoch().count()/1e9,
                      dt.count());
       }
    }
  else
    {
      logger_.log(EMANE::ERROR_LEVEL, " MHAL::RADIO %s not started, discard", __func__);
    }
}


void
EMANELTE::MHAL::MHALCommon::get_messages(RxMessages & rxMessages, timeval & tvSor)
{
  timing_.lockTime();

  // subframe start time since we were here last
  const timeval tvLastSfStart = timing_.getCurrSfTime();

  // subframe end time
  const timeval tvLastSfEnd = timing_.getNextSfTime();

  timeval tvNow, tvDiff;

  // actual time now
  gettimeofday(&tvNow, NULL);

  // deltaT the next subframe (end of last sub frame) and now
  timersub(&tvLastSfEnd, &tvNow, &tvDiff);

  const time_t timeToNextSfUsec = tvToUseconds(tvDiff);

  time_t overRunUsec = 0;

  // this is where we set the pace for the system pulse/tti
  // wait for the end of the subframe and return any rx messages
  if(timeToNextSfUsec >= 0)
    {
      // good, the end of this subframe time is now or still in the future
      //             
      //   |         |        |        |
      //   ^    ^    ^
      // begin now  end
      //

      if(timeToNextSfUsec > 0)
       {
         select(0, NULL, NULL, NULL, &tvDiff);
       }
      else
       {
         // fall thru
       }
    }
  else
    {
      // running late by a sf or more
      overRunUsec = abs(timeToNextSfUsec);      

      // a frame behind or more
      if(overRunUsec >= (10 * timing_.tsSfIntervalUsec()))
       {
         logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s last_sf_time %ld:%06ld, late by %ld usec",
                     __func__,
                     tvLastSfStart.tv_sec,
                     tvLastSfStart.tv_usec,
                     overRunUsec);

         // lets jump ahead
         while(overRunUsec > 0)
          {
            timing_.stepTime();

            overRunUsec -= timing_.tsSfIntervalUsec();
          }

         // clear all stale messages
         clearBins_i();

         timing_.unlockTime();

         // done, we'll be back
         return;
       }
    }

  // select bin based on the current working subframe time
  const auto bin = getMessageBin(tvLastSfStart, timing_.tsSfIntervalUsec());

  // work this subframe bin
  pendingMessageBins_[bin].lockBin();

  // only process current data
  if(overRunUsec <= timing_.tsSfIntervalUsec())
   {
     noiseWorker_safe(bin);

     // transfer to caller
     rxMessages = std::move(readyMessageBins_[bin].get());

#if 0
     for(const auto & rxMessage : rxMessages)
      {
        const auto & sfTime = rxMessage.rxControl_.sf_time_;

        // check the msg sf time
        timersub(&tvNow, &sfTime, &tvDiff);

        const auto dt = tvToMicroseconds(tvDiff); 

        if(abs(dt.count()) > timing_.tsSfIntervalUsec())
         {
           logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s bin %u, sf_time %ld:%06ld, dt %ld",
                       __func__,
                       bin,
                       sfTime.tv_sec,
                       sfTime.tv_usec,
                       dt.count());
         }
      }
#endif
   }

  // clear messages for this subframe bin
  pendingMessageBins_[bin].clear();

  readyMessageBins_[bin].clear();

  pendingMessageBins_[bin].unlockBin();

  // set the sor to the subframe time regardless on how late we are, caller will catch up
  // but needs to stay in sync with its internal state
  tvSor = tvLastSfStart;

  statisticManager_.updateHandoffMessages(bin, rxMessages.size());

  // now advance the subframe times, curr -> next, next -> next+1
  // and clear any stale data for the next subframe rx bin
  clearBin_i(timing_.stepTime());

  // get the process time for the calling thread for the time remaining in this subframe
  timersub(&tvNow, &tvLastSfStart, &tvDiff);

  statisticManager_.tallySubframeProcessTime(bin, tvDiff);

  timing_.unlockTime();
}

void
EMANELTE::MHAL::MHALCommon::clearReadyMessages_safe(const uint32_t bin)
{
  if(auto numOrphans = readyMessageBins_[bin].clearAndCheck())
    {
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, bin %u, purge %zu ready orphans",
                  __func__,
                  bin,
                  numOrphans);

      statisticManager_.updateOrphanedMessages(bin, numOrphans);
    }
}

void
EMANELTE::MHAL::MHALCommon::clearPendingMessages_safe(const uint32_t bin)
{
  if(auto numOrphans = pendingMessageBins_[bin].clearAndCheck())
    {
      logger_.log(EMANE::INFO_LEVEL, "MHAL::RADIO %s, bin %u, purge %zu pending orphans",
                  __func__,
                  bin,
                  numOrphans);

      statisticManager_.updateOrphanedMessages(bin, numOrphans);
    }
}


void
EMANELTE::MHAL::MHALCommon::clearBins_i()
{
  for(size_t bin = 0; bin < EMANELTE::NUM_SF_PER_FRAME; ++bin)
   {
     clearBin_i(bin);
   }
}

void
EMANELTE::MHAL::MHALCommon::clearBin_i(size_t bin)
{
   pendingMessageBins_[bin].lockBin();
   pendingMessageBins_[bin].clear();
   clearReadyMessages_safe(bin);
   pendingMessageBins_[bin].unlockBin();
}
