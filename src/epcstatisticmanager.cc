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

  // Bearer Table Definitions
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

  // EMMContext Table Definitions
  using EMMContextTable = OpenStatistic::Table<std::uint64_t>;

  EMMContextTable * pEMMContextTable_ = NULL;

  struct EMMContextEntry {
    std::uint64_t imsi_;
    std::uint32_t mme_ue_s1ap_id_;
    const char * state_;
    std::uint8_t procedure_transaction_id_;
    std::uint8_t attach_type_;
    std::uint32_t ue_ip_;
    std::uint32_t sgw_ctrl_fteid_teid_;

    EMMContextEntry(std::uint64_t imsi,
		    std::uint32_t mme_ue_s1ap_id,
		    const char * state,
		    std::uint8_t procedure_transaction_id,
		    std::uint8_t attach_type,
		    std::uint32_t ue_ip,
		    std::uint32_t sgw_ctrl_fteid_teid):
      imsi_{imsi},
      mme_ue_s1ap_id_{mme_ue_s1ap_id},
      state_{state},
      procedure_transaction_id_{procedure_transaction_id},
      attach_type_{attach_type},
      ue_ip_{ue_ip},
      sgw_ctrl_fteid_teid_{sgw_ctrl_fteid_teid}
    { }
  };

  using EMMContextKey = std::uint64_t;
  using EMMContextDB  = std::map<EMMContextKey, EMMContextEntry>;

  EMMContextDB EMMContextDB_;

  void updateEMMContextTable_(EMMContextTable * table,
			      EMMContextDB & db,
			      std::uint64_t imsi,
			      std::uint32_t mme_ue_s1ap_id,
			      const char * state,
			      std::uint8_t procedure_transaction_id,
			      std::uint8_t attach_type,
			      std::uint32_t ue_ip,
			      std::uint32_t sgw_ctrl_fteid_teid)
   {
     const EMMContextKey key{imsi};

     auto iter = db.find(key);

     const EMMContextEntry entry{imsi, mme_ue_s1ap_id, state, procedure_transaction_id, attach_type, ue_ip, sgw_ctrl_fteid_teid};

     const std::string sUEAddr = inet_ntoa(*(in_addr*)(&ue_ip));

     if(iter == db.end())
       {
	 // new entry
	 db.insert(std::make_pair(key, entry));

	 EMANELTE::addRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{key},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{procedure_transaction_id}},
				     OpenStatistic::Any{std::uint64_t{attach_type}},
				     OpenStatistic::Any{sUEAddr},
				     OpenStatistic::Any{std::uint64_t{sgw_ctrl_fteid_teid}}});
       }
     else
       {
	 // update entry
	 iter->second = entry;

	 EMANELTE::setRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{imsi},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{procedure_transaction_id}},
				     OpenStatistic::Any{std::uint64_t{attach_type}},
				     OpenStatistic::Any{sUEAddr},
				     OpenStatistic::Any{std::uint64_t{sgw_ctrl_fteid_teid}}});
       }
   }


  void deleteEMMContextTable_(EMMContextTable * table,
			      EMMContextDB & db,
			      std::uint64_t imsi)
   {
    const EMMContextKey key{imsi};

    auto iter = db.find(key);

    if(iter != db.end())
      {
        db.erase(iter);

        EMANELTE::delRowWithCheck_(table, key);
      }
   }

  // ECMContext Table Definitions
  using ECMContextTable = OpenStatistic::Table<std::uint64_t>;

  ECMContextTable * pECMContextTable_ = NULL;

  struct ECMContextEntry {
    std::uint64_t imsi_;
    std::uint32_t mme_ue_s1ap_id_;
    const char * state_;
    std::uint32_t enb_ue_s1ap_id_;
    // struct sctp_sndrcvinfo enb_sri
    bool eit_;

    ECMContextEntry(std::uint64_t imsi,
		    std::uint32_t mme_ue_s1ap_id,
		    const char * state,
		    std::uint32_t enb_ue_s1ap_id,
		    // struct sctp_sndrcvinfo enb_sri
		    bool eit):
      imsi_{imsi},
      mme_ue_s1ap_id_{mme_ue_s1ap_id},
      state_{state},
      enb_ue_s1ap_id_{enb_ue_s1ap_id},
      eit_{eit}
    { }
  };

  using ECMContextKey = std::uint64_t;
  using ECMContextDB  = std::map<ECMContextKey, ECMContextEntry>;

  ECMContextDB ECMContextDB_;

  void updateECMContextTable_(ECMContextTable * table,
			      ECMContextDB & db,
			      uint64_t imsi,
			      uint32_t mme_ue_s1ap_id,
			      const char * state,
			      uint32_t enb_ue_s1ap_id,
			      // struct sctp_sndrcvinfo enb_sri,
			      bool eit)
   {
     const ECMContextKey key{imsi};

     auto iter = db.find(key);

     const ECMContextEntry entry{imsi, mme_ue_s1ap_id, state, enb_ue_s1ap_id, eit};

     if(iter == db.end())
       {
	 // new entry
	 db.insert(std::make_pair(key, entry));

	 EMANELTE::addRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{key},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{enb_ue_s1ap_id}},
				     OpenStatistic::Any{eit}});
       }
     else
       {
	 // update entry
	 iter->second = entry;

	 EMANELTE::setRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{key},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{enb_ue_s1ap_id}},
				     OpenStatistic::Any{eit}});
       }
   }


  void deleteECMContextTable_(ECMContextTable * table,
			      ECMContextDB & db,
			      std::uint64_t imsi)
   {
    const ECMContextKey key{imsi};

    auto iter = db.find(key);

    if(iter != db.end())
      {
        db.erase(iter);

        EMANELTE::delRowWithCheck_(table, key);
      }
   }

  // ESMContext Table Definitions
  using ESMContextTable = OpenStatistic::Table<std::uint64_t>;

  ESMContextTable * pESMContextTable_ = NULL;

  struct ESMContextEntry {
    std::uint64_t imsi_;
    std::uint32_t mme_ue_s1ap_id_;
    uint8_t erab_id_;
    const char * state_;
    uint8_t qci_;
    uint32_t enb_fteid_teid_;
    uint32_t sgw_s1u_fteid_teid_;

    ESMContextEntry(uint64_t imsi,
		    uint32_t mme_ue_s1ap_id,
		    uint8_t erab_id,
		    const char * state,
		    uint8_t qci,
		    uint32_t enb_fteid_teid,
		    uint32_t sgw_s1u_fteid_teid):
      imsi_{imsi},
      mme_ue_s1ap_id_{mme_ue_s1ap_id},
      erab_id_{erab_id},
      state_{state},
      qci_{qci},
      enb_fteid_teid_{enb_fteid_teid},
      sgw_s1u_fteid_teid_{sgw_s1u_fteid_teid}
    { }
  };

  using ESMContextKey = std::uint64_t;
  using ESMContextDB  = std::map<ESMContextKey, ESMContextEntry>;

  ESMContextDB ESMContextDB_;

  void updateESMContextTable_(ESMContextTable * table,
			      ESMContextDB & db,
			      uint64_t imsi,
			      uint32_t mme_ue_s1ap_id,
			      uint8_t erab_id,
			      const char * state,
			      uint8_t qci,
			      uint32_t enb_fteid_teid,
			      uint32_t sgw_s1u_fteid_teid)
   {
     const ESMContextKey key{imsi};

     auto iter = db.find(key);

     const ESMContextEntry entry{imsi, mme_ue_s1ap_id, erab_id, state, qci, enb_fteid_teid, sgw_s1u_fteid_teid};

     if(iter == db.end())
       {
	 // new entry
	 db.insert(std::make_pair(key, entry));

	 EMANELTE::addRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{key},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{std::uint64_t{erab_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{qci}},
				     OpenStatistic::Any{std::uint64_t{enb_fteid_teid}},
				     OpenStatistic::Any{std::uint64_t{sgw_s1u_fteid_teid}}});
       }
     else
       {
	 // update entry
	 iter->second = entry;

	 EMANELTE::setRowWithCheck_(table,
				    key,
				    {OpenStatistic::Any{key},
				     OpenStatistic::Any{std::uint64_t{mme_ue_s1ap_id}},
				     OpenStatistic::Any{std::uint64_t{erab_id}},
				     OpenStatistic::Any{state},
				     OpenStatistic::Any{std::uint64_t{qci}},
				     OpenStatistic::Any{std::uint64_t{enb_fteid_teid}},
				     OpenStatistic::Any{std::uint64_t{sgw_s1u_fteid_teid}}});
       }
   }


  void deleteESMContextTable_(ESMContextTable * table,
			      ESMContextDB & db,
			      std::uint64_t imsi)
   {
    const ESMContextKey key{imsi};

    auto iter = db.find(key);

    if(iter != db.end())
      {
        db.erase(iter);

        EMANELTE::delRowWithCheck_(table, key);
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

  pEMMContextTable_ =
    registrar.registerTable<std::uint64_t>(
      "EMMContextTable",
      {"IMSI", "MMEUEID", "State", "ProcID", "AttType", "UEAddr",  "SGWTEID"},
      OpenStatistic::StatisticProperties::NONE,
      "EMM Context Table");

  pECMContextTable_ =
    registrar.registerTable<std::uint64_t>(
      "ECMContextTable",
      {"IMSI", "MMEUEID", "State", "ENBUEID", "EIT"},
      OpenStatistic::StatisticProperties::NONE,
      "ECM Context Table");

  pESMContextTable_ =
    registrar.registerTable<std::uint64_t>(
      "ESMContextTable",
      {"IMSI", "MMEUEID", "ERABID", "State", "QCI", "ENBTEID", "SGWTEID"},
      OpenStatistic::StatisticProperties::NONE,
      "ESM Context Table");

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


void EPCSTATS::addEMMContext(uint64_t imsi,
			     uint32_t mme_ue_s1ap_id,
			     const char * state,
			     uint8_t procedure_transaction_id,
			     uint8_t attach_type,
			     uint32_t ue_ip,
			     uint32_t sgw_ctrl_fteid_teid)
{
  updateEMMContextTable_(pEMMContextTable_,
			 EMMContextDB_,
			 imsi,
			 mme_ue_s1ap_id,
			 state,
			 procedure_transaction_id,
			 attach_type,
			 ue_ip,
			 sgw_ctrl_fteid_teid);
}


void EPCSTATS::delEMMContext(uint64_t imsi)
{
  deleteEMMContextTable_(pEMMContextTable_, EMMContextDB_, imsi);
}


void EPCSTATS::clearEMMContexts()
{
  pEMMContextTable_->clear();
  EMMContextDB_.clear();
}


void EPCSTATS::addECMContext(uint64_t imsi,
			     uint32_t mme_ue_s1ap_id,
			     const char * state,
			     uint32_t enb_ue_s1ap_id,
			     // struct sctp_sndrcvinfo enb_sri,
			     bool eit)
{
  updateECMContextTable_(pECMContextTable_,
			 ECMContextDB_,
			 imsi,
			 mme_ue_s1ap_id,
			 state,
			 enb_ue_s1ap_id,
			 // struct sctp_sndrcvinfo enb_sri,
			 eit);
}


void EPCSTATS::delECMContext(uint64_t imsi)
{
  deleteECMContextTable_(pECMContextTable_, ECMContextDB_, imsi);
}


void EPCSTATS::clearECMContexts()
{
  pECMContextTable_->clear();
  ECMContextDB_.clear();
}


void EPCSTATS::addESMContext(uint64_t imsi,
			     uint32_t mme_ue_s1ap_id,
			     uint8_t erab_id,
			     const char * state,
			     uint8_t qci,
			     uint32_t enb_fteid_teid,
			     uint32_t sgw_s1u_fteid_teid)
{
  updateESMContextTable_(pESMContextTable_,
			 ESMContextDB_,
			 imsi,
			 mme_ue_s1ap_id,
			 erab_id,
			 state,
			 qci,
			 enb_fteid_teid,
			 sgw_s1u_fteid_teid);
}


void EPCSTATS::delESMContext(uint64_t imsi)
{
  deleteESMContextTable_(pESMContextTable_, ESMContextDB_, imsi);
}


void EPCSTATS::clearESMContexts()
{
  pESMContextTable_->clear();
  ESMContextDB_.clear();
}
