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


#include "libemanelte/epcstatisticmanager.h"
#include "gtpcinterfacetypes.h"
#include "basicstatistichelper.h"
#include "iptrafficstats.h"

#include <ostatistic/service.h>
#include <ostatistic/statisticnumeric.h>
#include <ostatistic/table.h>
#include <map>
#include <string>
#include <sstream>
#include <utility>

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace {

  EMANELTE::IPTrafficTable * pDlIPTrafficTable_   = NULL;
  EMANELTE::IPTrafficTable * pUlIPTrafficTable_   = NULL;
  EMANELTE::IPTrafficTable * pDropIPTrafficTable_ = NULL;

  EMANELTE::IPTrafficDB dlTrafficDB_;
  EMANELTE::IPTrafficDB ulTrafficDB_;
  EMANELTE::IPTrafficDB dropTrafficDB_;

  using BearerTable = OpenStatistic::Table<std::string>;

  BearerTable * pBearerTable_ = NULL;

  struct BearerEntry {
   std::uint64_t teid_;
   std::uint32_t addr_;
   std::uint64_t imsi_;
   std::uint8_t  ebi_;

   BearerEntry(std::uint64_t teid, 
               std::uint32_t addr, 
               std::uint64_t imsi,
               std::uint8_t  ebi) :
    teid_{teid},
    addr_{addr},
    imsi_{imsi},
    ebi_{ebi}
   { }
  };

  using BearerKey = std::uint32_t;
  using BearerDB  = std::map<BearerKey, BearerEntry>;

  std::string keyToString(const BearerKey & key)
   {
     const std::string s1 = inet_ntoa(*(in_addr*)&(key));
     return s1;
   }

  BearerDB BearerDB_;

  void updateBearerTable_(BearerTable * table,
                          BearerDB & db,
                          std::uint32_t dst,
                          std::uint64_t teid,
                          std::uint32_t addr,
                          std::uint64_t imsi,
                          std::uint8_t ebi)
   {
    const BearerKey key{dst};

    const auto keyAsString = keyToString(key);
 
    const std::string sAddr = inet_ntoa(*(in_addr*)(&addr));

    auto iter = db.find(key);

    const BearerEntry entry{teid, addr, imsi, ebi};

    if(iter == db.end())
      {
        // new entry
        db.insert(std::make_pair(key, entry));

        EMANELTE::addRowWithCheck_(table, 
                                   keyAsString,
                                   {OpenStatistic::Any{keyAsString},           // 0 dst
                                    OpenStatistic::Any{EMANELTE::toHex(teid)}, // 1 teid
                                    OpenStatistic::Any{sAddr},                 // 2 addr
                                    OpenStatistic::Any{imsi},                  // 3 imsi
                                    OpenStatistic::Any{std::uint64_t{ebi}},    // 4 ebi
                                    OpenStatistic::Any{time(NULL)}});          // 5 Time
      }
    else
      {
        // update entry
        iter->second = entry;

        EMANELTE::setRowWithCheck_(table, 
                                   keyAsString,
                                   {OpenStatistic::Any{keyAsString},                         // 0 dst
                                    OpenStatistic::Any{EMANELTE::toHex(iter->second.teid_)}, // 1 teid
                                    OpenStatistic::Any{sAddr},                               // 2 addr
                                    OpenStatistic::Any{imsi},                                // 3 imsi
                                    OpenStatistic::Any{std::uint64_t{ebi}},                  // 4 ebi
                                    OpenStatistic::Any{time(NULL)}});                        // 5 Time
      }
   }


  void deleteBearerTable_(BearerTable * table,
                            BearerDB & db,
                            std::uint32_t dst)
   {
    const BearerKey key{dst};

    const auto keyAsString = keyToString(key);
 
    auto iter = db.find(key);

    if(iter != db.end())
      {
        db.erase(iter);

        EMANELTE::delRowWithCheck_(table, keyAsString);
      }
   }
}


void EPCSTATS::initialize(const std::string & endpoint)
{
  OpenStatistic::Service * service = OpenStatistic::Service::instance();

  OpenStatistic::Registrar & registrar = service->registrar();

  // variable length tables
  pDlIPTrafficTable_ =
    registrar.registerTable<std::string>(
      "DownlinkTrafficTable",
      {"Src", "Dst", "Count", "Bytes", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Downlink Traffic Table");

  pUlIPTrafficTable_ =
    registrar.registerTable<std::string>(
      "UplinkTrafficTable",
      {"Src", "Dst", "Count", "Bytes", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Downlink Traffic Table");

  pDropIPTrafficTable_ =
    registrar.registerTable<std::string>(
      "DownlinkDropTrafficTable",
      {"Src", "Dst", "Count", "Bytes", "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Downlink Traffic Table");

  pBearerTable_ =
    registrar.registerTable<std::string>(
      "BearerTable",
      {"Dst", "eNBTEID", "eNBAddr", "IMSI", "EBI",  "Time"},
      OpenStatistic::StatisticProperties::NONE,
      "Bearer Table");

  OpenStatistic::Service::instance()->start(endpoint);
}


void EPCSTATS::updateDstNotFound(uint32_t src, uint32_t dst, size_t numBytes)
{
  EMANELTE::updateIPTrafficTable_(pDropIPTrafficTable_, dropTrafficDB_, src, dst, numBytes);
}


void EPCSTATS::updateUplinkTraffic(uint32_t src, uint32_t dst, size_t numBytes)
{
  EMANELTE::updateIPTrafficTable_(pUlIPTrafficTable_, ulTrafficDB_, src, dst, numBytes);
}


void EPCSTATS::updateDownlinkTraffic(uint32_t src, uint32_t dst, size_t numBytes)
{
  EMANELTE::updateIPTrafficTable_(pDlIPTrafficTable_, dlTrafficDB_, src, dst, numBytes);
}


void EPCSTATS::addBearer(uint32_t dst, uint64_t teid, uint32_t addr, uint64_t imsi, uint8_t ebi)
{
  updateBearerTable_(pBearerTable_, BearerDB_, dst, teid, addr, imsi, ebi);
}


void EPCSTATS::delBearer(uint32_t dst)
{
  deleteBearerTable_(pBearerTable_, BearerDB_, dst);
}

