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


#include "radiostatisticmanager.h"
#include "basicstatistichelper.h"
#include "emane/platformserviceprovider.h"
#include "emane/statistictableexception.h"


EMANE::StatisticTableLabels EMANE::Models::LTE::RESOURCE_BLOCK_STATISTIC_TABLE_LABELS
{
  "Frequency", // 0  key
  "0.1",       // 1  counter
  "0.2",       // Subframe 0 slot 1
  "1.1",       // Subframe 0 slot 2
  "1.2",       // Subframe 1 slot 1
  "2.1",       // Subframe 1 slot 2
  "2.2",       // etc.
  "3.1",
  "3.2",
  "4.1",
  "4.2",
  "5.1",
  "5.2",
  "6.1",
  "6.2",
  "7.1",
  "7.2",
  "8.1",
  "8.2",
  "9.1",
  "9.2"
};


namespace {

enum RX_FREQ_IDX{ RX_FREQ_COUNT = 0,
                  RX_FREQ_NF    = 1,
                  RX_FREQ_SERR  = 2,
                  RX_FREQ_PASS  = 3,
                  RX_FREQ_DROP  = 4 };
}


EMANE::Models::LTE::RadioStatisticManager::RadioStatisticManager(EMANE::NEMId id,
                                                                 EMANE::PlatformServiceProvider * pPlatformService,
                                                                 std::set<EMANELTE::MHAL::CHANNEL_TYPE> rxChannelMessageSet,
                                                                 std::set<EMANELTE::MHAL::CHANNEL_TYPE> txChannelMessageSet) :
  id_{id},
  pPlatformService_{pPlatformService},
  rxChannelMessageSet_{rxChannelMessageSet},
  txChannelMessageSet_{txChannelMessageSet}
{
  for(auto channel_type : rxChannelMessageSet_)
    {
      rxFrequencyCounts_.insert(std::make_pair(channel_type, FrameFrequencyCount()));
    }

  for(auto channel_type : txChannelMessageSet_)
    {
      txFrequencyCounts_.insert(std::make_pair(channel_type, FrameFrequencyCount()));
    }
}


void
EMANE::Models::LTE::RadioStatisticManager::registerStatistics(EMANE::StatisticRegistrar & statisticRegistrar)
{
  pRxFrequencyTable_ =
    statisticRegistrar.registerTable<std::uint64_t>(
      "RxFrequency",
      {"Frequency", "NumSamples", "NoiseFloordBm", "SpectrumErrors", "PassSegments", "DropSegments", "Time"},
      EMANE::StatisticProperties::NONE,
      "Noise floor (dBm) vs. Frequency.");
}


void
EMANE::Models::LTE::RadioStatisticManager::updateFrequencies(const EMANE::FrequencySet & rx, const EMANE::FrequencySet & tx)
{
  for(auto channel_type : txChannelMessageSet_)
    {
      txFrequencyTables_[channel_type]->clear();

      for(auto freq : tx)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::DEBUG_LEVEL,
                                  "RadioModel %03hu %s adding channeltype=%d TX table freq=%lu.",
                                  id_,
                                  __func__,
                                  channel_type,
                                  freq);

          std::vector<EMANE::Any> v{
            EMANE::Any{std::uint64_t{freq}},                           // 0  key resource block frequency
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},  // 2 slots for each subframe
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},                                                                                                                                                                                     EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}}};

          try
           {
             txFrequencyTables_[channel_type]->addRow(freq, v);
           }
          catch(const EMANE::StatisticTableException & ex)
           {
            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    EMANE::ERROR_LEVEL,
                                    "RadioModel %03hu %s %s error, tx freq %lu",
                                    id_,
                                    __func__,
                                    ex.what(),
                                    freq);
           }
        }
    }

  for(auto channel_type : rxChannelMessageSet_)
    {
      rxFrequencyTables_[channel_type]->clear();;

      for(auto freq : rx)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::DEBUG_LEVEL,
                                  "RadioModel %03hu %s adding channeltype=%d RX table freq=%lu.",
                                  id_,
                                  __func__,
                                  channel_type,
                                  freq);

          std::vector<EMANE::Any> v{
            EMANE::Any{std::uint64_t{freq}},                           // 0  key resource block frequency
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},  // 2 slots for each subframe
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}},
            EMANE::Any{std::uint64_t{}}, EMANE::Any{std::uint64_t{}}};

          try
           {
             rxFrequencyTables_[channel_type]->addRow(freq, v);
           }
          catch(const EMANE::StatisticTableException & ex)
           {
            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    EMANE::ERROR_LEVEL,
                                    "RadioModel %03hu %s %s error, rx freq %lu",
                                    id_,
                                    __func__,
                                    ex.what(),
                                    freq);
           }
        }
    }
}


void
EMANE::Models::LTE::RadioStatisticManager::updateTableCounts(
   uint32_t tti,
   const EMANELTE::MHAL::ChannelMessage & channel_message,
   bool transmit)
{
  EMANELTE::MHAL::CHANNEL_TYPE channel_type = channel_message.channel_type();

  ChannelMap & counts_map = transmit ? txFrequencyCounts_ : rxFrequencyCounts_;

  auto count_iter = counts_map.find(channel_type);

  ChannelFrequencyStatTableMap & table_map = transmit ? txFrequencyTables_ : rxFrequencyTables_;

  auto table_iter = table_map.find(channel_type);

  uint32_t slot_index = (tti % 10) * 2;

  if(count_iter == counts_map.end() || table_iter == table_map.end())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::ERROR_LEVEL,
                              "RadioModel %03hu %s %s channel_type=%d slot_index=%d not expected.",
                              id_,
                              __func__,
                              transmit ? "TX" : "RX",
                              channel_type,
                              slot_index);

      return;
    }

  for(int i=0; i<channel_message.resource_block_frequencies_slot1_size(); ++i)
    {
      uint64_t freq = channel_message.resource_block_frequencies_slot1(i);

      count_iter->second[slot_index][freq]++;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             EMANE::DEBUG_LEVEL,
                             "RadioModel %03hu %s %s channel_type=%d slot_index=%d freq=%lu count=%lu.",
                             id_,
                             __func__,
                             transmit ? "TX" : "RX",
                             channel_type,
                             slot_index,
                             freq,
                             count_iter->second[slot_index][freq]);

      try
       {
         table_iter->second->setCell(freq,
                                     slot_index + 1,  // stat table colums index is +1 to avoid key column
                                     EMANE::Any{count_iter->second[slot_index][freq]});
       }
      catch(const EMANE::StatisticTableException & ex) 
       {
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                 EMANE::ERROR_LEVEL,
                                 "RadioModel %03hu %s %s error, freq %lu, col %u",
                                 id_,
                                 __func__,
                                 ex.what(),
                                 freq,
                                 slot_index + 1);
       }
    }

  slot_index++;

  for(int i=0; i<channel_message.resource_block_frequencies_slot2_size(); ++i)
    {
      uint64_t freq = channel_message.resource_block_frequencies_slot2(i);

      count_iter->second[slot_index][freq]++;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             EMANE::DEBUG_LEVEL,
                             "RadioModel %03hu %s %s channel_type=%d slot_index=%d freq=%lu count=%lu.",
                             id_,
                             __func__,
                             transmit ? "TX" : "RX",
                             channel_type,
                             slot_index,
                             freq,
                             count_iter->second[slot_index][freq]);

      try
       {
        table_iter->second->setCell(freq,
                                    slot_index + 1,  // stat table colums index is +1 to avoid key column
                                    EMANE::Any{count_iter->second[slot_index][freq]});
       }
      catch(const EMANE::StatisticTableException & ex)
       {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                EMANE::ERROR_LEVEL,
                                "RadioModel %03hu %s %s error, freq %lu, col %u",
                                id_,
                                __func__,
                                ex.what(),
                                freq,
                                slot_index + 1);
      }
   }
}


void EMANE::Models::LTE::RadioStatisticManager::updateRxFrequencyAvgNoiseFloor(EMANELTE::FrequencyHz frequency,  double noiseFloor_mW)
{
  auto iter = rxFrequencyMap_.find(frequency);

  if(iter == rxFrequencyMap_.end())
    {
      iter = rxFrequencyMap_.insert(std::make_pair(frequency, RxFrequencyTableEntry{1, EMANELTE::MW_TO_DB(noiseFloor_mW), 0, 0, 0})).first;

      pRxFrequencyTable_->addRow(frequency,
                                 {EMANE::Any{frequency},                         // 0
                                  EMANE::Any{uint64_t{1}},                       // 2 num samples in average
                                  EMANE::Any{EMANELTE::MW_TO_DB(noiseFloor_mW)}, // 1 avg noise floor dBm
                                  EMANE::Any{uint32_t{0}},                       // 2
                                  EMANE::Any{uint32_t{0}},                       // 3
                                  EMANE::Any{uint32_t{0}},                       // 4
                                  EMANE::Any{time(NULL)}});                      // 5
    }
  else
    {
      uint64_t & count = std::get<RX_FREQ_COUNT>(iter->second);
      double & sum_dB = std::get<RX_FREQ_NF>(iter->second);

      count  += 1;
      sum_dB += EMANELTE::MW_TO_DB(noiseFloor_mW);

      pRxFrequencyTable_->setRow(frequency,
                                 {EMANE::Any{frequency},
                                  EMANE::Any{count},                                 // num samples in average
                                  EMANE::Any{sum_dB/count},                          // 1 avg noise floor dBm
                                  EMANE::Any{std::get<RX_FREQ_SERR>(iter->second)},
                                  EMANE::Any{std::get<RX_FREQ_PASS>(iter->second)},
                                  EMANE::Any{std::get<RX_FREQ_DROP>(iter->second)},
                                  EMANE::Any{time(NULL)}});
    }
}


void EMANE::Models::LTE::RadioStatisticManager::updateRxFrequencySpectrumError(EMANELTE::FrequencyHz frequency)
{
  auto iter = rxFrequencyMap_.find(frequency);

  if(iter == rxFrequencyMap_.end())
    {
      // no update, only updateRxFrequencyAvgNoiseFloor initializes
      return;
    }

  pRxFrequencyTable_->setCell(frequency,
                              RX_FREQ_SERR+1,
                              EMANE::Any{++std::get<RX_FREQ_SERR>(iter->second)});
}


void EMANE::Models::LTE::RadioStatisticManager::updateRxFrequencyPass(EMANELTE::FrequencyHz frequency)
{
  auto iter = rxFrequencyMap_.find(frequency);

  if(iter == rxFrequencyMap_.end())
    {
      // no update, only updateRxFrequencyAvgNoiseFloor initializes
      return;
    }

  pRxFrequencyTable_->setCell(frequency,
                              RX_FREQ_PASS+1,
                              EMANE::Any{++std::get<RX_FREQ_PASS>(iter->second)});
}


void EMANE::Models::LTE::RadioStatisticManager::updateRxFrequencyDrop(EMANELTE::FrequencyHz frequency)
{
  auto iter = rxFrequencyMap_.find(frequency);

  if(iter == rxFrequencyMap_.end())
    {
      // no update, only updateRxFrequencyAvgNoiseFloor initializes
      return;
    }

  pRxFrequencyTable_->setCell(frequency,
                              RX_FREQ_DROP+1,
                              EMANE::Any{++std::get<RX_FREQ_DROP>(iter->second)});
}
