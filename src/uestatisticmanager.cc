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


#include "libemanelte/uestatisticmanager.h"
#include "iptrafficstats.h"
#include "channelcounter.h"
#include <ostatistic/service.h>
#include <ostatistic/statisticnumeric.h>
#include <ostatistic/table.h>
#include <array>
#include <math.h>
#include <string>
#include <map>
#include <sys/time.h>

namespace {

  OpenStatistic::Table<std::uint64_t> * pNodeInfoTable_       = NULL;
  OpenStatistic::Table<std::uint64_t> * pPhySearchStateTable_ = NULL;
  OpenStatistic::Table<std::uint64_t> * pCellTable_           = NULL;
  OpenStatistic::Table<std::uint64_t> * pMacTable_            = NULL;
  OpenStatistic::Table<std::uint64_t> * pULGrantRntiTable_    = NULL;
  OpenStatistic::Table<std::uint64_t> * pPDCCHRntiTable_      = NULL;
  OpenStatistic::Table<std::uint64_t> * pPDSCHRntiTable_      = NULL;


  float report_interval_secs_ = 0;

  const std::array<std::string, 8> phySearchStateNames = { "CellSearchPass",   // 0
                                                           "CellSearchFail",   // 1
                                                           "MIBSearchPass",    // 2
                                                           "MIBSearchFail",    // 3
                                                           "SFNSearchPass",    // 4
                                                           "SFNSearchFail",    // 5
                                                           "SyncSearchPass",   // 6
                                                           "SyncSearchFail" }; // 7

  std::array<std::uint64_t, 8> phySearchStateCounts_;

  void setPhySearchStateCell_i(size_t state, bool bStatus)
   {
     if(pPhySearchStateTable_)
      {
        // state plus pass/fail offset
        const size_t idx = state + (bStatus ? 0 : 1);

        // row, column, value
        pPhySearchStateTable_->setCell(idx, 1, OpenStatistic::Any{++phySearchStateCounts_[idx]});
        pPhySearchStateTable_->setCell(idx, 2, OpenStatistic::Any{time(NULL)});
      }
   }

  
  EMANELTE::IPTrafficTable * pDlIPTrafficTable_ = NULL;
  EMANELTE::IPTrafficTable * pUlIPTrafficTable_ = NULL;

  EMANELTE::IPTrafficDB dlTrafficDB_;
  EMANELTE::IPTrafficDB ulTrafficDB_;

  GrantRNTICounter   ulGrantDB_;
  ChannelRNTICounter pdcchDB_;
  ChannelRNTICounter pdschDB_;

  // track mac totals over time
  // carrier <rxPkts, rxErrors, txPkts, txErrors> 
  std::map<size_t, std::tuple<size_t, size_t, size_t, size_t>> macTableSums_; 
#define GET_RXPKT(x)  std::get<0>(x)
#define GET_RXERR(x)  std::get<1>(x)
#define GET_TXPKT(x)  std::get<2>(x)
#define GET_TXERR(x)  std::get<3>(x)

}


void UESTATS::initialize(float report_interval_secs)
{
  OpenStatistic::Service * service = OpenStatistic::Service::instance();

  OpenStatistic::Registrar & registrar = service->registrar();

  report_interval_secs_ = report_interval_secs;

  // phy state search table
  pPhySearchStateTable_ =
    registrar.registerTable<std::uint64_t>(
      "PhySearchStateTable",
      {"State", "Count", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Phy Search State Table");

  // fixed size table, setup all rows
  for(size_t n = 0; n < phySearchStateNames.size(); ++n)
    {
       pPhySearchStateTable_->addRow(
        n,
        {OpenStatistic::Any{phySearchStateNames[n]}, // 0 State
         OpenStatistic::Any{std::uint64_t{}},        // 1 Count
         OpenStatistic::Any{std::uint64_t{}}});      // 2 Time
    }

  // node info table
  pNodeInfoTable_ =
    registrar.registerTable<std::uint64_t>(
      "NodeInfoTable",
      {"RNTI", "IpAddress", "Netmask", "RRCState", "EMMState", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Node Info Table");

  // fixed size table, setup 1 row
  pNodeInfoTable_->addRow(
        0,
        {OpenStatistic::Any{std::uint64_t{}},   // 0 RNTI
         OpenStatistic::Any{"N/A"},             // 1 IpAddress
         OpenStatistic::Any{"N/A"},             // 2 Netmask 
         OpenStatistic::Any{"N/A"},             // 3 RRCState
         OpenStatistic::Any{"N/A"},             // 4 EMMState
         OpenStatistic::Any{std::uint64_t{}}}); // 5 Time


  // variable length cell table
  pCellTable_ =
    registrar.registerTable<std::uint64_t>(
      "CellTable",
      {"Id", "EARFCN", "Peak SNR", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Cell Table");


  // mac table
  pMacTable_ =
    registrar.registerTable<std::uint64_t>(
      "MACTable",
      {"Carrier", "ULPkts", "ULPktSum", "ULErr", "ULErrSum", "ULKbps", "ULPkts", "DLPktSum", "DLErr", "DLErrSum", "DLKbps", "ULBuffer", "DLRetxAvg", "ULRetxAvg", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Mac Table");

   // variable length tables
   pDlIPTrafficTable_ =
     registrar.registerTable<std::string>(
       "DownlinkTrafficTable",
       {"Src", "Dst", "Count", "Bytes", "Time"},
       OpenStatistic::StatisticProperties::NONE,
       "Downlink Traffic Table");

   // variable length tables
   pUlIPTrafficTable_ =
     registrar.registerTable<std::string>(
       "UplinkTrafficTable",
       {"Src", "Dst", "Count", "Bytes", "Time"},
       OpenStatistic::StatisticProperties::NONE,
       "Uplink Traffic Table");

  pULGrantRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "ULGrantTable",
      {"RNTI", "Count", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Uplink Grants tx per RNTI");

  pPDCCHRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "PDCCHGrantTable",
      {"RNTI", "Pass", "Fail", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Dowmlink PDCCH Grants rx per RNTI");

  pPDSCHRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "PDSCHGrantTable",
      {"RNTI", "Pass", "Fail", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Dowmlink PDSCH Grants rx per RNTI");

}


void UESTATS::enterCellSearch(const UESTATS::Cells & cells, std::uint32_t earfcn)
{
  const time_t ts = time(NULL);

  setPhySearchStateCell_i(0, !cells.empty());

  // update cell table
  if(pCellTable_)
    {
      pCellTable_->clear();

      size_t n = 0;
      for(auto & cell : cells)
       {
         pCellTable_->addRow(
          n++,
          {OpenStatistic::Any{cell.first},
           OpenStatistic::Any{uint64_t{earfcn}},
           OpenStatistic::Any{cell.second},
           OpenStatistic::Any{ts}});
       }
    }
}


void UESTATS::enterMibSearch(bool bStatus)
{
  setPhySearchStateCell_i(2, bStatus);
}


void UESTATS::enterSysFrameSearch(bool bStatus)
{
  setPhySearchStateCell_i(4, bStatus);
}


void UESTATS::enterSyncSearch(bool bStatus)
{
  setPhySearchStateCell_i(6, bStatus);
}


void UESTATS::setCrnti(std::uint16_t crnti)
{
  if(pNodeInfoTable_)
   {
     pNodeInfoTable_->setCell(0, 0, OpenStatistic::Any{static_cast<uint64_t>(crnti)});
     pNodeInfoTable_->setCell(0, 5, OpenStatistic::Any{time(NULL)});
   }
}


void UESTATS::setIpAddress(uint32_t ip_addr, uint32_t netmask)
{
  if(pNodeInfoTable_)
   {
     pNodeInfoTable_->setCell(0, 1, OpenStatistic::Any{inet_ntoa(*((in_addr*)&ip_addr))});  // addr
     pNodeInfoTable_->setCell(0, 2, OpenStatistic::Any{inet_ntoa(*((in_addr*)&netmask))});  // netmask
     pNodeInfoTable_->setCell(0, 5, OpenStatistic::Any{time(NULL)});                        // timestamp
   }
}


void UESTATS::setRRCState(const char * state)
{
  if(pNodeInfoTable_)
   {
     pNodeInfoTable_->setCell(0, 3, OpenStatistic::Any{state});      // state
     pNodeInfoTable_->setCell(0, 5, OpenStatistic::Any{time(NULL)}); // timestamp
   }
}


void UESTATS::setEMMState(const char * state)
{
  if(pNodeInfoTable_)
   {
     pNodeInfoTable_->setCell(0, 4, OpenStatistic::Any{state});      // state
     pNodeInfoTable_->setCell(0, 5, OpenStatistic::Any{time(NULL)}); // timestamp
   }
}


void UESTATS::setMACMetrics(const UESTATS::MACMetrics & metrics)
{
  if(pMacTable_)
   {
     pMacTable_->clear();

     for(size_t n = 0; n < metrics.size(); ++n)
      {
       const auto & metric = metrics[n];

       auto & sums = macTableSums_[n];

       pMacTable_->addRow(
         n,
         {OpenStatistic::Any{n},
          OpenStatistic::Any{metric.tx_pkts_},
          OpenStatistic::Any{GET_TXPKT(sums) += metric.tx_pkts_},
          OpenStatistic::Any{metric.tx_errors_},
          OpenStatistic::Any{GET_TXERR(sums) += metric.tx_errors_},
          OpenStatistic::Any{roundf(metric.tx_brate_kbps_/report_interval_secs_)},
          OpenStatistic::Any{metric.rx_pkts_},
          OpenStatistic::Any{GET_RXPKT(sums) += metric.rx_pkts_},
          OpenStatistic::Any{metric.rx_errors_},
          OpenStatistic::Any{GET_RXERR(sums) += metric.rx_errors_},
          OpenStatistic::Any{roundf(metric.rx_brate_kbps_/report_interval_secs_)},
          OpenStatistic::Any{metric.ul_buffer_},
          OpenStatistic::Any{metric.dl_retx_avg_},
          OpenStatistic::Any{metric.ul_retx_avg_},
          OpenStatistic::Any{time(NULL)}});
      }
   }
}



void UESTATS::updateUplinkTraffic(uint32_t src, uint32_t dst, size_t numBytes)
{
  updateIPTrafficTable_(pUlIPTrafficTable_, ulTrafficDB_, src, dst, numBytes);
}


void UESTATS::updateDownlinkTraffic(uint32_t src, uint32_t dst, size_t numBytes)
{
  updateIPTrafficTable_(pDlIPTrafficTable_, dlTrafficDB_, src, dst, numBytes);
}


void UESTATS::putULGrant(uint16_t rnti)
{
  auto iter = ulGrantDB_.find(rnti);

  if(iter == ulGrantDB_.end())
   {
     pULGrantRntiTable_->addRow(rnti,
                                {OpenStatistic::Any{uint64_t{rnti}},
                                 OpenStatistic::Any{uint64_t{1}},
                                 OpenStatistic::Any{time(NULL)}});

     ulGrantDB_.insert(std::make_pair(rnti, 1));
   }
  else
   {
     pULGrantRntiTable_->setRow(rnti,
                                {OpenStatistic::Any{uint64_t{rnti}},
                                 OpenStatistic::Any{uint64_t{++iter->second}},
                                 OpenStatistic::Any{time(NULL)}});
   }
}


void UESTATS::getPDCCH(uint16_t rnti, bool bPass)
{
  updateChannelCounter_i(rnti, bPass, pPDCCHRntiTable_, pdcchDB_);
}


void UESTATS::getPDSCH(uint16_t rnti, bool bPass)
{
  updateChannelCounter_i(rnti, bPass, pPDSCHRntiTable_, pdschDB_);
}
