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


#include "libemanelte/enbstatisticmanager.h"
#include "channelcounter.h"
#include <ostatistic/service.h>
#include <ostatistic/statisticnumeric.h>
#include <ostatistic/table.h>
#include <array>
#include <math.h>
#include <string>
#include <sys/time.h>

namespace {
  OpenStatistic::Table<std::uint64_t> * pNodeInfoTable_      = NULL;
  OpenStatistic::Table<std::uint64_t> * pMacTable_           = NULL;
  OpenStatistic::Table<std::uint64_t> * pDLGrantRntiTable_   = NULL;
  OpenStatistic::Table<std::uint64_t> * pPUCCHRntiTable_     = NULL;
  OpenStatistic::Table<std::uint64_t> * pPUSCHRntiTable_     = NULL;

  float report_interval_secs_ = 0;

  GrantRNTICounter   dlGrantDB_;
  ChannelRNTICounter pucchDB_;
  ChannelRNTICounter puschDB_;

  // track mac totals over time
  // carrier <rxPkts, rxErrors, txPkts, txErrors> 
  std::map<size_t, std::tuple<size_t, size_t, size_t, size_t>> macTableSums_; 
#define GET_RXPKT(x)  std::get<0>(x)
#define GET_RXERR(x)  std::get<1>(x)
#define GET_TXPKT(x)  std::get<2>(x)
#define GET_TXERR(x)  std::get<3>(x)

}


void ENBSTATS::initialize(float report_interval_secs)
{
  OpenStatistic::Service * service = OpenStatistic::Service::instance();

  OpenStatistic::Registrar & registrar = service->registrar();

  report_interval_secs_ = report_interval_secs;

  // node info table
  pNodeInfoTable_ =
    registrar.registerTable<std::uint64_t>(
      "NodeInfoTable",
      {"S1State", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Node Info Table");

  // fixed size table, setup 1 row
  pNodeInfoTable_->addRow(
        0,
        {OpenStatistic::Any{"N/A"},             // 0 S1State
         OpenStatistic::Any{std::uint64_t{}}}); // 1 Time

  // mac table
  pMacTable_ =
    registrar.registerTable<std::uint64_t>(
      "MACTable",
      {"RNTI", "State", "DLPkts", "DLPktSum", "DLErr", "DLErrSum", "DLKbps", "ULPkts", "ULPktSum", "ULErr", "ULErrSum", "ULKbps", "ULBuf", "DLBuf", "DLCQI", "DLRI", "DLPMI", "PHR", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Mac Table");

  // variable length dl rnti table
  pDLGrantRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "DLGrantTable",
      {"RNTI", "Count", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Downlink Grants tx per RNTI");

  pPUCCHRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "PUCCHGrantTable",
      {"RNTI", "Pass", "Fail", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Uplink PUCCH Grants rx per RNTI");

  pPUSCHRntiTable_ =
    registrar.registerTable<std::uint64_t>(
      "PUSCHGrantTable",
      {"RNTI", "Pass", "Fail", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Uplink PUSCH Grants rx per RNTI");
}


void ENBSTATS::setS1State(const char * state)
{
  if(pNodeInfoTable_)
    {
      pNodeInfoTable_->setCell(0, 0, OpenStatistic::Any{state});      // state
      pNodeInfoTable_->setCell(0, 1, OpenStatistic::Any{time(NULL)}); // timestamp
    }
}


void ENBSTATS::setMACMetrics(const ENBSTATS::MACMetrics & metrics)
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
             {OpenStatistic::Any{metric.rnti_},
              OpenStatistic::Any{metric.state_},
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
              OpenStatistic::Any{metric.dl_buffer_},
              OpenStatistic::Any{metric.dl_cqi_},
              OpenStatistic::Any{metric.dl_ri_},
              OpenStatistic::Any{metric.dl_pmi_},
              OpenStatistic::Any{metric.phr_},
              OpenStatistic::Any{metric.ts_}});
       }
   }
}


void ENBSTATS::putDLGrant(uint16_t rnti)
{
  auto iter = dlGrantDB_.find(rnti);

  if(iter == dlGrantDB_.end())
   {
     pDLGrantRntiTable_->addRow(rnti,
                                {OpenStatistic::Any{uint64_t{rnti}},
                                 OpenStatistic::Any{uint64_t{1}},
                                 OpenStatistic::Any{time(NULL)}});

     dlGrantDB_.insert(std::make_pair(rnti, 1));
   }
  else
   {
     pDLGrantRntiTable_->setRow(rnti,
                                {OpenStatistic::Any{uint64_t{rnti}},
                                 OpenStatistic::Any{uint64_t{++iter->second}},
                                 OpenStatistic::Any{time(NULL)}});
   }
}


void ENBSTATS::getPUCCH(uint16_t rnti, bool bPass)
{
  updateChannelCounter_i(rnti, bPass, pPUCCHRntiTable_, pucchDB_);
}

void ENBSTATS::getPUSCH(uint16_t rnti, bool bPass)
{
  updateChannelCounter_i(rnti, bPass, pPUSCHRntiTable_, puschDB_);
}

