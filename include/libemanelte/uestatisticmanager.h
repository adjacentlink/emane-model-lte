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

#ifndef EMANELTE_UESTATISTICMANAGER_H
#define EMANELTE_UESTATISTICMANAGER_H

#include <string>
#include <stdint.h>
#include <arpa/inet.h>
#include <map>
#include <vector>
#include <string>

namespace UESTATS {
      void initialize(float report_interval_secs);

      typedef std::map<uint64_t, float> Cells;
      void enterCellSearch(const Cells & cells, uint32_t earfcn);

      void enterMibSearch(bool bStatus);
      void enterSysFrameSearch(bool bStatus);
      void enterSyncSearch(bool bStatus);

      void setCrnti(uint16_t crtni);
      void setIpAddress(uint32_t ip_addr, uint32_t netmask);
      void setRRCState(const char * state);
      void setEMMState(const char * state);

      struct MACMetric { 
        const uint64_t tx_pkts_;
        const uint64_t tx_errors_;
        const float tx_brate_kbps_;
        const uint64_t rx_pkts_;
        const uint64_t rx_errors_;
        const float rx_brate_kbps_;
        const uint64_t ul_buffer_;
        const float dl_retx_avg_;
        const float ul_retx_avg_;
         MACMetric(int tx_pkts, 
                    int tx_errors,
                    int tx_brate,
                    int rx_pkts,
                    int rx_errors,
                    int rx_brate,
                    int ul_buffer,
                    float dl_retx_avg,
                    float ul_retx_avg):
          tx_pkts_(tx_pkts),
          tx_errors_(tx_errors),
          tx_brate_kbps_(tx_brate/1e3),
          rx_pkts_(rx_pkts),
          rx_errors_(rx_errors), 
          rx_brate_kbps_(rx_brate/1e3),
          ul_buffer_(ul_buffer),
          dl_retx_avg_(dl_retx_avg),
          ul_retx_avg_(ul_retx_avg)
        {}
      };
      typedef std::vector<MACMetric> MACMetrics;

      void setMACMetrics(const MACMetrics & metrics);

      void updateUplinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void updateDownlinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void putULGrant(uint16_t rnti);

      void getPDCCH(uint16_t rnti, bool bPass);

      void getPDSCH(uint16_t rnti, bool bPass);
}

#endif // EMANELTE_UESTATISTICMANAGER_H
