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


#include "statisticmanager.h"
#include "utils.h"
#include "timinginfo.h"
#include <algorithm>

namespace {
  enum US_PKT_IDX{ US_PKT_SF      = 0, 
                   US_PKT_LATE    = 1, 
                   US_PKT_ORPH    = 2,
                   US_PKT_EXPIRED = 3, 
                   US_PKT_ENQUE   = 4, 
                   US_PKT_DEQUE   = 5, 
                   US_PKT_SINR    = 6,
                   US_PKT_SLOTERR = 7,
                   US_PKT_HANDOFF = 8,
                   US_PKT_RDYAVG  = 9,
                   US_PKT_RDYMAX  = 10,
                   US_PKT_OTADLY  = 11 };
}

EMANELTE::MHAL::StatisticManager::StatisticManager(EMANE::Application::Logger & logger) :
  StatisticHelper{logger},
  logger_(logger),
  quarterSubframeIntervalMicrosecs_{25000},
  subframeProcessTimes_{{}},
  orphanedSubframeReceiveCounts_{{}},
  lateSubframeReceiveCounts_{{}},
  expiredSubframeReceiveCounts_{{}},
  enqueuedSubframeReceiveCounts_{{}},
  dequeuedSubframeReceiveCounts_{{}},
  dropSINRSubframeReceiveCounts_{{}},
  handoffSubframeReceiveCounts_{{}},
  slotErrorSubframeReceiveCounts_{{}},
  noiseProcLateCounts_{{}},
  rxNoiseProcessDelay_{{}},
  rxUpstreamPacketOtaDelay_{{{}}},
  rxUpstreamReadyDepth_{{{}}}
{
}

void
EMANELTE::MHAL::StatisticManager::start(const std::string & endpoint, std::uint32_t sf_interval_msec)
{
  OpenStatistic::Service * service = OpenStatistic::Service::instance();

  OpenStatistic::Registrar & registrar = service->registrar();

  // Register statistic tables
  pSubframeReceiveCountsTable_ =
    registrar.registerTable<std::uint64_t>(
      "SubframeReceiveCounts",
      {"Subframe", "RxLate", "Orphans", "Expired", "Enqueue", "Dequeue", "DropSINR", "SlotErr", "Handoff", "ReadyAvg", "ReadyMax", "OTADelay" },
      OpenStatistic::StatisticProperties::NONE,
      "Count of receive subframes organized by subframe number.");

  pSubframeProcessTimesTable_ =
    registrar.registerTable<std::uint64_t>(
      "SubframeProcessTimes",
      {"Subframe", "0.25", "0.50", "0.75", "1.00", "1.25", "1.50", "1.75", "2.00", ">2", "NoiseProcBusy", "NoiseProcDelay"},
      OpenStatistic::StatisticProperties::NONE,
      "Tally, by percentage of a subframe, the amount of time needed to perform subframe work.");

  pReceptionTable_ =
    registrar.registerTable<std::uint64_t>(
      "ReceptionTable",
      {"Src", "SINRAvg", "NoiseFloorAvgdBm", "Samples", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Receptions for each nem");

  for(uint64_t sf=0; sf<EMANELTE::NUM_SF_PER_FRAME; ++sf)
    {
      addRowWithCheck(__func__,
                      pSubframeReceiveCountsTable_,
                      sf,
                      {OpenStatistic::Any{uint64_t{sf}},  // 0 Subframe 
                       OpenStatistic::Any{uint64_t{}},    // 1 Late
                       OpenStatistic::Any{uint64_t{}},    // 2 Orphaned
                       OpenStatistic::Any{uint64_t{}},    // 3 Expired
                       OpenStatistic::Any{uint64_t{}},    // 4 Enqueue
                       OpenStatistic::Any{uint64_t{}},    // 5 Dequeue
                       OpenStatistic::Any{uint64_t{}},    // 6 DropSINR
                       OpenStatistic::Any{uint64_t{}},    // 7 SlotErr
                       OpenStatistic::Any{uint64_t{}},    // 8 Handoff
                       OpenStatistic::Any{0.0},           // 9 ReadyAvg
                       OpenStatistic::Any{uint64_t{}},    // 10 ReadyMax
                       OpenStatistic::Any{0.0}});         // 11 OTADelay

      addRowWithCheck(__func__,
                      pSubframeProcessTimesTable_,
                      sf,
                      {OpenStatistic::Any{sf},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{std::uint64_t{}},
                       OpenStatistic::Any{0.0}});
    }

  quarterSubframeIntervalMicrosecs_ = sf_interval_msec * 1000 / 4,

  service->start(endpoint);
}


void
EMANELTE::MHAL::StatisticManager::stop()
{
  OpenStatistic::Service::instance()->stop();
}


EMANELTE::MHAL::StatisticManager::~StatisticManager()
{}


void
EMANELTE::MHAL::StatisticManager::updateLateMessages(size_t subframe, std::uint64_t increment)
{
  lateSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_LATE, OpenStatistic::Any{lateSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateOrphanedMessages(size_t subframe, std::uint64_t increment)
{
  orphanedSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_ORPH, OpenStatistic::Any{orphanedSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateExpiredMessages(size_t subframe, std::uint64_t increment)
{
  expiredSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_EXPIRED, OpenStatistic::Any{expiredSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateEnqueuedMessages(size_t subframe, std::uint64_t increment)
{
  enqueuedSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_ENQUE, OpenStatistic::Any{enqueuedSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateDequeuedMessages(size_t subframe, std::uint64_t increment)
{
  dequeuedSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_DEQUE, OpenStatistic::Any{dequeuedSubframeReceiveCounts_[subframe]});

  auto & count = std::get<0>(rxUpstreamReadyDepth_[subframe]);
  auto & max   = std::get<1>(rxUpstreamReadyDepth_[subframe]);
  auto & sum   = std::get<2>(rxUpstreamReadyDepth_[subframe]);

  count += 1;
  sum   += increment;
  max = std::max(max, increment);

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_RDYAVG, OpenStatistic::Any{sum/count});
  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_RDYMAX, OpenStatistic::Any{max});
}


void
EMANELTE::MHAL::StatisticManager::updateDropSINRMessages(size_t subframe, std::uint64_t increment)
{
  dropSINRSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_SINR, OpenStatistic::Any{dropSINRSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateHandoffMessages(size_t subframe, std::uint64_t increment)
{
  handoffSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_HANDOFF, OpenStatistic::Any{handoffSubframeReceiveCounts_[subframe]});
}


void
EMANELTE::MHAL::StatisticManager::updateSlotErrorMessages(size_t subframe, std::uint64_t increment)
{
  slotErrorSubframeReceiveCounts_[subframe] += increment;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_SLOTERR, OpenStatistic::Any{slotErrorSubframeReceiveCounts_[subframe]});
}



void EMANELTE::MHAL::StatisticManager::updateRxPacketOtaDelay(size_t subframe, double seconds)
{
  auto & count = std::get<0>(rxUpstreamPacketOtaDelay_[subframe]);
  auto & sum   = std::get<1>(rxUpstreamPacketOtaDelay_[subframe]);

  count += 1;
  sum   += seconds;

  setCellWithCheck(__func__, pSubframeReceiveCountsTable_, subframe, US_PKT_OTADLY, OpenStatistic::Any{1e3 * (sum / count)});
}


void
EMANELTE::MHAL::StatisticManager::tallySubframeProcessTime(size_t subframe, const timeval & tv_diff)
{
  auto bin = static_cast<size_t>(tvToUseconds(tv_diff) / quarterSubframeIntervalMicrosecs_);

  bin = std::min(bin, NUM_SUBFRAME_TIME_BINS-1);

  auto counts = ++subframeProcessTimes_[subframe][bin];

  // bin+1 is the table column
  setCellWithCheck(__func__, pSubframeProcessTimesTable_, subframe, bin+1, OpenStatistic::Any{counts});

  // late bins
  if(bin > 3)
   {
     auto & item = noiseProcLateCounts_[subframe];
     item  += 1;

     setCellWithCheck(__func__, pSubframeProcessTimesTable_, subframe, NUM_SUBFRAME_TIME_BINS + 1, OpenStatistic::Any{item});
   }
}



void EMANELTE::MHAL::StatisticManager::updateNoiseProcessDelay(size_t subframe, double seconds)
{
  auto & count = std::get<0>(rxNoiseProcessDelay_[subframe]);
  auto & sum   = std::get<1>(rxNoiseProcessDelay_[subframe]);

  count += 1;
  sum   += seconds;

  setCellWithCheck(__func__, pSubframeProcessTimesTable_, subframe, NUM_SUBFRAME_TIME_BINS + 2, OpenStatistic::Any{1e3 * (sum / count)});
}



void EMANELTE::MHAL::StatisticManager::updateReceptionTable(const EMANELTE::MHAL::StatisticManager::ReceptionInfoMap & receptionInfoMap)
{
  // xxx clear table of stale entries

  for(auto info : receptionInfoMap)
    {
      const auto & src = info.first;
      const auto & sig = std::get<0>(info.second);
      const auto & nf  = std::get<1>(info.second);
      const auto & increment = std::get<2>(info.second);

      const std::uint64_t key = src;

      auto iter = receptionTableMap_.find(key);

      // new entry
      if(iter == receptionTableMap_.end())
       {
         iter = receptionTableMap_.insert(std::make_pair(key, time(NULL))).first;

         addRowWithCheck(__func__,
                         pReceptionTable_,
                         key,
                         {OpenStatistic::Any{std::uint64_t{src}},
                          OpenStatistic::Any{sig-nf},
                          OpenStatistic::Any{nf},
                          OpenStatistic::Any{increment},
                          OpenStatistic::Any{iter->second}});
       }
      else
       {
         iter->second = time(NULL);

         setRowWithCheck(__func__,
                         pReceptionTable_,
                         key,
                         {OpenStatistic::Any{std::uint64_t{src}},
                          OpenStatistic::Any{sig-nf},
                          OpenStatistic::Any{nf},
                          OpenStatistic::Any{increment},
                          OpenStatistic::Any{iter->second}});
       }
    }
}
