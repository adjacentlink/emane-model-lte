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

#ifndef EMANELTE_MHAL_PENDINGMESSAGEBIN_H
#define EMANELTE_MHAL_PENDINGMESSAGEBIN_H

#include <tuple>
#include <list>

#include "libemanelte/mhal.h"
#include "mhalphy.h"
#include "statisticmanager.h"
#include "utils.h"


namespace EMANELTE {
namespace MHAL {
  // pending rx message, moves to ready after noise processing
#define PendingMessage_Data(x)      std::get<0>((x))
#define PendingMessage_RxControl(x) std::get<1>((x))
#define PendingMessage_OtaInfo(x)   std::get<2>((x))
#define PendingMessage_TxControl(x) std::get<3>((x))

  using PendingMessage = std::tuple<Data,              // opaque data
                                    RxControl,         // rx control
                                    PHY::OTAInfo,      // emane ota info
                                    TxControlMessage>; // tx control

  using PendingMessages = std::list<PendingMessage>;

  using SegmentTimeSpan = std::tuple<EMANE::TimePoint,  // sor
                                     EMANE::TimePoint,  // eor
                                     size_t>;           // num segments

  // frequency, time span
  using SegmentSpans = std::map<std::uint64_t, SegmentTimeSpan>;

  class PendingMessageBin {
   public:
    PendingMessageBin() :
      binTime_{0,0}
    {
      EMANELTE::MHAL::init_mutex(&mutex_);
    }

    void clear();

    size_t clearAndCheck();

    // purge and msgs that have expired.
    void add(const timeval & binTime,
             uint32_t bin,
             const PendingMessage & msg,
             StatisticManager & statisticManager);

    // a msg is composed of multiple segments each with a unique frequency
    // find the min sor and max eor for each frequency
    // this will be used to consult the spectrum monitor later
    SegmentSpans getSegmentSpans();

    PendingMessages & get();

    void unlockBin();

    void lockBin();

  private:
    pthread_mutex_t     mutex_;
    PendingMessages     pending_;
    timeval             binTime_;
  };
}
}

#endif
