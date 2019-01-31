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

#ifndef EMANELTE_MHAL_STATISTICMANAGER_H
#define EMANELTE_MHAL_STATISTICMANAGER_H

#include "ltedefs.h"
#include <cstdint>
#include <array>
#include <map>
#include <ostatistic/service.h>
#include <ostatistic/exception.h>
#include "statistichelper.h"

namespace EMANELTE {
namespace MHAL {

class StatisticManager : public StatisticHelper
{
public:
  StatisticManager(EMANE::Application::Logger & logger);

  ~StatisticManager();

  void start(const std::string & endpoint, std::uint32_t sf_interval_msec);

  void stop();

  void updateOrphanedMessages(size_t subframe, std::uint64_t increment = 1);
  void updateLateMessages(size_t subframe, std::uint64_t increment = 1);
  void updateExpiredMessages(size_t subframe, std::uint64_t increment = 1);
  void updateEnqueuedMessages(size_t subframe, std::uint64_t increment = 1);
  void updateDequeuedMessages(size_t subframe, std::uint64_t increment = 1);
  void updateDropSINRMessages(size_t subframe, std::uint64_t increment = 1);
  void updateHandoffMessages(size_t subframe, std::uint64_t increment = 1);
  void updateSlotErrorMessages(size_t subframe, std::uint64_t increment = 1);
  
  void updateRxPacketOtaDelay(size_t subframe, double seconds);
  void updateNoiseProcessDelay(size_t subframe, double seconds);

  // src, <sinr, noiseFloor, samples>
  using ReceptionInfoData = std::tuple<double, double, size_t>;
  using ReceptionInfoMap  = std::map<std::uint16_t, ReceptionInfoData>;

  void updateReceptionTable(const ReceptionInfoMap & receptionInfoMap);

  void tallySubframeProcessTime(const size_t subframe, const timeval & tv_diff, bool bWasBlocked);
  
private:
  EMANE::Application::Logger & logger_;

  static const size_t NUM_SUBFRAME_TIME_BINS = 9;

  uint32_t quarterSubframeIntervalMicrosecs_;

  std::array<std::array<std::uint64_t, NUM_SUBFRAME_TIME_BINS>, EMANELTE::NUM_SF_PER_FRAME> subframeProcessTimes_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> orphanedSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> lateSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> expiredSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> enqueuedSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> dequeuedSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> dropSINRSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> handoffSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> slotErrorSubframeReceiveCounts_;
  std::array<std::uint64_t, EMANELTE::NUM_SF_PER_FRAME> noiseProcLateCounts_;

  std::array<std::tuple<size_t, double>, EMANELTE::NUM_SF_PER_FRAME> rxNoiseProcessDelay_;
  std::array<std::tuple<size_t, double>, EMANELTE::NUM_SF_PER_FRAME> rxUpstreamPacketOtaDelay_;
  std::array<std::tuple<size_t, size_t, double>, EMANELTE::NUM_SF_PER_FRAME> rxUpstreamReadyDepth_;

  // src
  using ReceptionTableMap = std::map<std::uint64_t, time_t>;

  ReceptionTableMap receptionTableMap_;

  OpenStatistic::Table<std::uint64_t> * pSubframeProcessTimesTable_;
  OpenStatistic::Table<std::uint64_t> * pSubframeReceiveCountsTable_;
  OpenStatistic::Table<std::uint64_t> * pReceptionTable_;
};

  
}
}

#endif // EMANELTE_STATISTICMANAGER_H
