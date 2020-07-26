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


#ifndef EMANELTE_MHAL_RADIOSTATISTICMANAGER_H
#define EMANELTE_MHAL_RADIOSTATISTICMANAGER_H

#include "ltedefs.h"
#include "emane/controls/frequencyofinterestcontrolmessage.h"
#include "emane/statisticnumeric.h"
#include "emane/statistictable.h"
#include "emane/platformserviceprovider.h"
#include "emane/registrar.h"
#include "libemanelte/txcontrolmessage.pb.h"
#include <map>
#include <array>
#include <set>


namespace EMANE
{
namespace Models
{
namespace LTE
{
    extern EMANE::StatisticTableLabels RESOURCE_BLOCK_STATISTIC_TABLE_LABELS;

    class RadioStatisticManager
    {
    public:
      RadioStatisticManager(EMANE::NEMId id,
                            EMANE::PlatformServiceProvider * pPlatformService,
                            std::set<EMANELTE::MHAL::CHANNEL_TYPE> rxChannelMessageSet,
                            std::set<EMANELTE::MHAL::CHANNEL_TYPE> txChannelMessageSet);

      virtual ~RadioStatisticManager() {}

      virtual void updateTxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl) = 0;

      virtual void updateRxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl, 
                                       const EMANELTE::CarriersOfInterest & carriersOfInterest) = 0;

      void updateFrequencies(const EMANE::FrequencySet & rx, const EMANE::FrequencySet & tx);

      void updateRxFrequencyAvgNoiseFloor(EMANELTE::FrequencyHz frequency, double noiseFloor_mW);
      void updateRxFrequencySpectrumError(EMANELTE::FrequencyHz frequency);
      void updateRxFrequencyPass(EMANELTE::FrequencyHz frequency);
      void updateRxFrequencyDrop(EMANELTE::FrequencyHz frequency);

    protected:
      EMANE::NEMId id_;
      EMANE::PlatformServiceProvider * pPlatformService_;
    
      std::set<EMANELTE::MHAL::CHANNEL_TYPE> rxChannelMessageSet_;
      std::set<EMANELTE::MHAL::CHANNEL_TYPE> txChannelMessageSet_;

      using SubframeFrequencyCounts = std::map<std::uint64_t, std::uint64_t>;

      using FrameFrequencyCount = std::array<SubframeFrequencyCounts, EMANELTE::NUM_SLOTS_PER_FRAME>;

      using ChannelMap = std::map<EMANELTE::MHAL::CHANNEL_TYPE, FrameFrequencyCount>;

      ChannelMap rxFrequencyCounts_;
      ChannelMap txFrequencyCounts_;

      using ChannelFrequencyStatTableMap = std::map<EMANELTE::MHAL::CHANNEL_TYPE, EMANE::StatisticTable<std::uint64_t, std::greater<EMANE::Any>> *>;

      ChannelFrequencyStatTableMap rxFrequencyTables_;
      ChannelFrequencyStatTableMap txFrequencyTables_;

      // num samples, avg noise floor errors sum, pass sum, drop sum
      using RxFrequencyTableEntry = std::tuple<uint64_t, double, uint32_t, uint32_t, uint32_t>;
      using RxFrequencyTableMap   = std::map<EMANELTE::FrequencyHz, RxFrequencyTableEntry>;
      RxFrequencyTableMap rxFrequencyMap_;
      EMANE::StatisticTable<std::uint64_t> * pRxFrequencyTable_;

      void registerStatistics(EMANE::StatisticRegistrar & statisticRegistrar);

      void updateTableCounts(const uint32_t tti, const EMANELTE::MHAL::ChannelMessage & channel_message, bool transmit);
    };
}
}
}

#endif //EMANELTE_MHAL_RADIOSTATISTICMANAGER_H
