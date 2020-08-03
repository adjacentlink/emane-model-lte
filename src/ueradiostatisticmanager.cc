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

#include "ueradiostatisticmanager.h"


EMANE::Models::LTE::UERadioStatisticManager::UERadioStatisticManager(EMANE::NEMId id,
                                                                 EMANE::PlatformServiceProvider * pPlatformService) :
  RadioStatisticManager{
    id,
    pPlatformService,
    std::set<EMANELTE::MHAL::CHANNEL_TYPE>{
      EMANELTE::MHAL::CHAN_PBCH,
      EMANELTE::MHAL::CHAN_PCFICH,
      EMANELTE::MHAL::CHAN_PDCCH,
      EMANELTE::MHAL::CHAN_PDSCH,
      EMANELTE::MHAL::CHAN_PHICH,
      EMANELTE::MHAL::CHAN_PMCH},
    std::set<EMANELTE::MHAL::CHANNEL_TYPE>{
      EMANELTE::MHAL::CHAN_PRACH,
      EMANELTE::MHAL::CHAN_PUCCH,
      EMANELTE::MHAL::CHAN_PUSCH}}
{}


void
EMANE::Models::LTE::UERadioStatisticManager::registerStatistics(EMANE::Registrar & registrar)
{
  auto & statisticRegistrar = registrar.statisticRegistrar();

  RadioStatisticManager::registerStatistics(statisticRegistrar);

  // downlink receive
  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PBCH] =                       \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPBCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PBCH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PCFICH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPCFICHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PCFICH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PDCCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPDCCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PDCCH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PDSCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPDSCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PDSCH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PHICH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPHICHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PHICH Resource Block receive counts");

  rxFrequencyTables_[EMANELTE::MHAL::CHAN_PMCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "DownlinkRxPMCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Downlink PMCH Resource Block receive counts");

  // uplink transmit
  txFrequencyTables_[EMANELTE::MHAL::CHAN_PRACH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkTxPRACHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PRACH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PUCCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkTxPUCCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PUCCH Resource Block transmit counts");

  txFrequencyTables_[EMANELTE::MHAL::CHAN_PUSCH] = \
    statisticRegistrar.registerTable<std::uint64_t, std::greater<EMANE::Any>>(
      "UplinkTxPUSCHFrequencyCounts",
      RESOURCE_BLOCK_STATISTIC_TABLE_LABELS,
      EMANE::StatisticProperties::NONE,
      "Uplink PUSCH Resource Block transmit counts");
}



void
EMANE::Models::LTE::UERadioStatisticManager::updateTxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl)
{
  for(const auto & carrier : txControl.carriers())
   {
     if(carrier.uplink().has_prach())
      {
        updateTableCounts(txControl.tti_tx(), carrier.uplink().prach(), true);
      }

     for(int i = 0; i < carrier.uplink().pucch_size(); ++i)
      {
        updateTableCounts(txControl.tti_tx(), carrier.uplink().pucch(i), true);
      }

     for(int i = 0; i < carrier.uplink().pusch_size(); ++i)
      {
        updateTableCounts(txControl.tti_tx(), carrier.uplink().pusch(i), true);
      }
   }
}


void
EMANE::Models::LTE::UERadioStatisticManager::updateRxTableCounts(const EMANELTE::MHAL::TxControlMessage & txControl,
                                                                 const EMANELTE::FrequencySet & carriersOfInterest)
{
  for(const auto & carrier : txControl.carriers())
   {
     if(carriersOfInterest.count(carrier.frequency_hz()))
      {
        updateTableCounts(txControl.tti_tx(), carrier.downlink().pcfich(), false);

        if(carrier.downlink().has_pbch())
         {
           updateTableCounts(txControl.tti_tx(), carrier.downlink().pbch(), false);
         }

        for(int i = 0; i < carrier.downlink().phich_size(); ++i)
         {
           updateTableCounts(txControl.tti_tx(), carrier.downlink().phich(i), false);
         }
  
        for(int i = 0; i < carrier.downlink().pdcch_size(); ++i)
         {
           updateTableCounts(txControl.tti_tx(), carrier.downlink().pdcch(i), false);
         }

       for(int i = 0; i < carrier.downlink().pdsch_size(); ++i)
        {
          updateTableCounts(txControl.tti_tx(), carrier.downlink().pdsch(i), false);
        }

       if(carrier.downlink().has_pmch())
        {
         updateTableCounts(txControl.tti_tx(), carrier.downlink().pmch(), false);
        }
      }
   }
}
