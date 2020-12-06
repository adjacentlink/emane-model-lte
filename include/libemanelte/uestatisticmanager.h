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

      struct GWMetrics {
          const float dl_mbps_;
          const float ul_mbps_;
          GWMetrics(float dl, float ul):
            dl_mbps_(dl),
            ul_mbps_(ul)
          {}
        };
      void setGWMetrics(const GWMetrics & metrics);

      struct RLCQueueMetrics {
        int type_;
        int64_t capacity_;
        uint64_t currSize_;
        uint64_t highWater_;
        uint64_t numCleared_;
        uint64_t numPush_;
        uint64_t numPushFail_;
        uint64_t numPop_;
        uint64_t numPopFail_;

        RLCQueueMetrics() :
         type_(-1),
         capacity_(0),
         currSize_(0),
         highWater_(0),
         numCleared_(0),
         numPush_(0),
         numPushFail_(0),
         numPop_(0),
         numPopFail_(0)
        { }

        RLCQueueMetrics(int type,
                        int64_t capacity, 
                        uint64_t currSize,
                        uint64_t highWater,
                        uint64_t numCleared,
                        uint64_t numPush,
                        uint64_t numPushFail,
                        uint64_t numPop,
                        uint64_t numPopFail) :
         type_(type),
         capacity_(capacity),
         currSize_(currSize),
         highWater_(highWater),
         numCleared_(numCleared),
         numPush_(numPush),
         numPushFail_(numPushFail),
         numPop_(numPop),
         numPopFail_(numPopFail)
        { }

        std::string typeToString() const
         {
           return type_ == 0 ? "TM" :
                  type_ == 1 ? "UM" :
                  type_ == 2 ? "AM" : "N/A";
         }
      };

      typedef std::vector<RLCQueueMetrics> RLCQueueMetricsList;

      // see lib/src/upper/rlc.cc dl == rx, ul == tx
      struct RLCMetrics {
          const float * dl_mbps_;
          const float * ul_mbps_;
          const RLCQueueMetricsList & qmetrics_;
          const float * dl_mrb_mbps_;
          const RLCQueueMetricsList & mrb_qmetrics_;

          RLCMetrics(const float * dl, 
                     const float * ul, 
                     const RLCQueueMetricsList & qmetrics,
                     const float * dl_mrb, 
                     const RLCQueueMetricsList & mrb_qmetrics) :
            dl_mbps_(dl),
            ul_mbps_(ul),
            qmetrics_(qmetrics),
            dl_mrb_mbps_(dl_mrb),
            mrb_qmetrics_(mrb_qmetrics)
          { }
        };
      void setRLCMetrics(const RLCMetrics & metrics);

      struct MACMetrics { 
        const uint64_t tx_pkts_;
        const uint64_t tx_errors_;
        const float tx_brate_kbps_;
        const uint64_t rx_pkts_;
        const uint64_t rx_errors_;
        const float rx_brate_kbps_;
        const uint64_t ul_buffer_;
        const float dl_retx_avg_;
        const float ul_retx_avg_;
         MACMetrics(int tx_pkts, 
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
      void setMACMetrics(const MACMetrics & metrics);

     struct PHYMetrics {
        const float sync_ta_us_;
        const float sync_cfo_; 
        const float sync_sfo_; 
        const float dl_noise_;
        const float dl_sinr_;
        const float dl_rsrp_;
        const float dl_rsrq_;
        const float dl_rssi_;
        const float dl_turbo_iters_;
        const float dl_mcs_;
        const float dl_pathloss_;
        const float dl_mabr_mbps_;
        const float ul_mcs_; 
        const float ul_power_;
        const float ul_mabr_mbps_;
 
        PHYMetrics(float sync_ta_us, 
                   float sync_cfo,
                   float sync_sfo,
                   float dl_noise,
                   float dl_sinr,
                   float dl_rsrp,
                   float dl_rsrq,
                   float dl_rssi,
                   float dl_turbo_iters,
                   float dl_mcs,
                   float dl_pathloss,
                   float dl_mabr_mbps,
                   float ul_mcs,
                   float ul_power,
                   float ul_mabr_mbps):
          sync_ta_us_(sync_ta_us),
          sync_cfo_(sync_cfo),
          sync_sfo_(sync_sfo),
          dl_noise_(dl_noise),
          dl_sinr_(dl_sinr),
          dl_rsrp_(dl_rsrp),
          dl_rsrq_(dl_rsrq),
          dl_rssi_(dl_rssi),
          dl_turbo_iters_(dl_turbo_iters),
          dl_mcs_(dl_mcs),
          dl_pathloss_(dl_pathloss),
          dl_mabr_mbps_(dl_mabr_mbps),
          ul_mcs_(ul_mcs),
          ul_power_(ul_power),
          ul_mabr_mbps_(ul_mabr_mbps)
         {}
       };
      void setPHYMetrics(const PHYMetrics & metrics);

      void updateUplinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void updateDownlinkTraffic(uint32_t src, uint32_t dst, size_t numBytes);

      void putULGrant(uint16_t rnti);

      void getPDCCH(uint16_t rnti, bool bPass);

      void getPDSCH(uint16_t rnti, bool bPass);
}

#endif // EMANELTE_UESTATISTICMANAGER_H
