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

#ifndef EMANELTE_ENBSTATISTICMANAGER_H
#define EMANELTE_ENBSTATISTICMANAGER_H

#include <stdint.h>
#include <arpa/inet.h>
#include <string>
#include <vector>
#include <map>
#include <sys/time.h>

namespace ENBSTATS {
      void initialize(float report_interval_secs);

      void setS1State(const char * state);

      struct MACMetric { 
        uint64_t rnti_;
        uint64_t tx_pkts_;
        uint64_t tx_errors_;
        float tx_brate_kbps_;
        uint64_t rx_pkts_;
        uint64_t rx_errors_;
        float rx_brate_kbps_;
        uint64_t ul_buffer_;
        uint64_t dl_buffer_;
        float dl_cqi_;
        float dl_ri_;
        float dl_pmi_;
        float phr_;
        std::string state_;
        time_t ts_;
        MACMetric(uint16_t rnti,
                   int tx_pkts,
                   int tx_errors,
                   int tx_brate,
                   int rx_pkts,
                   int rx_errors,
                   int rx_brate,
                   int ul_buffer,
                   int dl_buffer,
                   float dl_cqi,
                   float dl_ri,
                   float dl_pmi,
                   float phr,
                   const std::string & state):
          rnti_(rnti),
          tx_pkts_(tx_pkts),
          tx_errors_(tx_errors),
          tx_brate_kbps_(tx_brate/1e3),
          rx_pkts_(rx_pkts),
          rx_errors_(rx_errors), 
          rx_brate_kbps_(rx_brate/1e3),
          ul_buffer_(ul_buffer),
          dl_buffer_(dl_buffer),
          dl_cqi_(dl_cqi),
          dl_ri_(dl_ri),
          dl_pmi_(dl_pmi),
          phr_(phr),
          state_(state),
          ts_(time(NULL))
        {}
      };
      typedef std::vector<MACMetric> MACMetrics;

      void setMACMetrics(const MACMetrics & metrics);

      struct RLCQueueMetric {
        int type_;
        int64_t capacity_; // can be -1
        uint64_t currSize_;
        uint64_t highWater_;
        uint64_t numCleared_;
        uint64_t numPush_;
        uint64_t numPushFail_;
        uint64_t numPop_;
        uint64_t numPopFail_;

        RLCQueueMetric() :
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

        RLCQueueMetric(int type,
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

      // metrics for each bearer (LCID)
      struct RLCBearerMetric {
          float tx_mbps_;
          float rx_mbps_;
          RLCQueueMetric qmetric_;

          RLCBearerMetric() :
            tx_mbps_(0),
            rx_mbps_(0)
          { }

          // see lib/src/upper/rlc.cc dl == rx, ul == tx
          RLCBearerMetric(const float dl, 
                          const float ul, 
                          const RLCQueueMetric & qmetric) :
            tx_mbps_(ul),
            rx_mbps_(dl),
            qmetric_(qmetric)
          { }
        };

      // metrics for each mrb bearer (LCID)
      struct RLCMRBBearerMetric {
          float tx_mbps_;
          RLCQueueMetric qmetric_;

          RLCMRBBearerMetric() :
            tx_mbps_(0)
          { }

          RLCMRBBearerMetric(const float dl, 
                             const RLCQueueMetric & qmetric) :
            tx_mbps_(dl),
            qmetric_(qmetric)
          { }
        };

      // list of bearers (LCID's)
      typedef std::vector<RLCBearerMetric> RLCBearerMetrics;

      // list of mrb bearers (LCID's)
      typedef std::vector<RLCMRBBearerMetric> RLCMRBBearerMetrics;

      typedef std::pair<RLCBearerMetrics, RLCMRBBearerMetrics> RLCMetric;

      // user (RNTI) to bearer metrics
      typedef std::map<uint16_t, RLCMetric> RLCMetrics;

      void setRLCMetrics(const RLCMetrics & metrics);

      void putDLGrant(uint16_t rnti);

      void getPUCCH(uint16_t rnti, bool bPass);

      void getPUSCH(uint16_t rnti, bool bPass);
}

#endif // EMANELTE_ENBSTATISTICMANAGER_H
