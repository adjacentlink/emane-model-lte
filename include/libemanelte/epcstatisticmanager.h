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

#ifndef EMANELTE_EPCSTATISTICMANAGER_H
#define EMANELTE_EPCSTATISTICMANAGER_H

#include <stdint.h>
#include <arpa/inet.h>
#include <string>

namespace EPCSTATS {
      void initialize(const std::string & endpoint);

      void updateDstNotFound(uint32_t src, uint32_t dst, size_t numBytes);

      void updateUplinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void updateDownlinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void addBearer(uint32_t dst, uint64_t teid, uint32_t addr, uint64_t imsi, uint8_t ebi);
      void delBearer(uint32_t dst);

     void addEMMContext(uint64_t imsi,
			uint32_t mme_ue_s1ap_id,
			const char * state,
			uint8_t procedure_transaction_id,
			uint8_t attach_type,
			uint32_t ue_ip,
			uint32_t sgw_ctrl_fteid_teid);
      void delEMMContext(uint64_t imsi);
      void clearEMMContexts();

      void addECMContext(uint64_t imsi,
			 uint32_t mme_ue_s1ap_id,
			 const char * state,
			 uint32_t enb_ue_s1ap_id,
			 // struct sctp_sndrcvinfo enb_sri,
			 bool eit);
      void delECMContext(uint64_t imsi);
      void clearECMContexts();

      void addESMContext(uint64_t imsi,
			 uint32_t mme_ue_s1ap_id,
			 uint8_t erab_id,
			 const char * state,
			 uint8_t qci,
			 uint32_t enb_fteid_teid,
			 uint32_t sgw_s1u_fteid_teid);
      void delESMContext(uint64_t imsi);
      void clearESMContexts();
}

#endif // EMANELTE_EPCSTATISTICMANAGER_H
