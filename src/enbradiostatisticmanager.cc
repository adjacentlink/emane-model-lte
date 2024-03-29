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

#include "enbradiostatisticmanager.h"
#include "libemanelte/otacommon.pb.h"

EMANE::Models::LTE::ENBRadioStatisticManager::ENBRadioStatisticManager(EMANE::NEMId id,
                                                                   EMANE::PlatformServiceProvider * pPlatformService) :
  RadioStatisticManager{
    id,
    pPlatformService,
    std::set<EMANELTE::MHAL::CHANNEL_TYPE>{
      EMANELTE::MHAL::CHAN_PRACH,
      EMANELTE::MHAL::CHAN_PUCCH,
      EMANELTE::MHAL::CHAN_PUSCH},
    std::set<EMANELTE::MHAL::CHANNEL_TYPE>{
      EMANELTE::MHAL::CHAN_PBCH,
      EMANELTE::MHAL::CHAN_PCFICH,
      EMANELTE::MHAL::CHAN_PDCCH,
      EMANELTE::MHAL::CHAN_PDSCH,
      EMANELTE::MHAL::CHAN_PHICH,
      EMANELTE::MHAL::CHAN_PMCH}}
{}


void
EMANE::Models::LTE::ENBRadioStatisticManager::registerStatistics(EMANE::Registrar & registrar)
{
  auto & statisticRegistrar = registrar.statisticRegistrar();

  RadioStatisticManager::registerStatistics(statisticRegistrar);

  // downlink transmit
  txFrequencyTables_[EMANELTE::MHAL::CHAN_PBCH] =                       \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPBCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PBCH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PCFICH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPCFICHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PCFICH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PDCCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPDCCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PDCCH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PDSCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPDSCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PDSCH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PHICH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPHICHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PHICH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PMCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkTxPMCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PMCH Resource Block transmit counts");

  // uplink receive
  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PRACH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkRxPRACHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PRACH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PUCCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkRxPUCCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PUCCH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PUSCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkRxPUSCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PUSCH Resource Block receive counts");  
}




void
EMANE::Models::LTE::ENBRadioStatisticManager::updateTxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl)
{
  for(const auto & carrier : txControl.carriers())
   {
     updateTableCounts(txControl.tti_tx(), carrier.downlink().pcfich(), true);

     if(carrier.downlink().has_pbch())
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().pbch(), true);
      }

     for(int i = 0; i < carrier.downlink().phich_size(); ++i)
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().phich(i), true);
      }

     for(int i = 0; i < carrier.downlink().pdcch_size(); ++i)
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().pdcch(i), true);
      }

     for(int i = 0; i < carrier.downlink().pdsch_size(); ++i)
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().pdsch(i), true);
      }

     if(carrier.downlink().has_pmch())
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().pmch(), true);
      }
   }
}

void
EMANE::Models::LTE::ENBRadioStatisticManager::updateRxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl,
                                                                  const EMANELTE::FrequencySet & carriersOfInterest)
{
  for(const auto & carrier : txControl.carriers())
   {
     if(carriersOfInterest.count(carrier.frequency_hz()))
      {
        if(carrier.uplink().has_prach())
         {
          updateTableCounts(txControl.tti_tx(), carrier.uplink().prach(), false);
         }

        for(int i = 0; i < carrier.uplink().pucch_size(); ++i)
         {
           updateTableCounts(txControl.tti_tx(), carrier.uplink().pucch(i), false);
         }

        for(int i = 0; i < carrier.uplink().pusch_size(); ++i)
         {
           updateTableCounts(txControl.tti_tx(), carrier.uplink().pusch(i), false);
         }
      }
   }
}
