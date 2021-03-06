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


#ifndef EMANELTE_MHAL_H
#define EMANELTE_MHAL_H

#include <vector>
#include <string>
#include <sys/time.h>
#include "libemanelte/otacommon.pb.h"
#include "libemanelte/sinrtester.h"


namespace EMANELTE {
namespace MHAL {
  typedef std::string Data;

  struct RxData {
    uint16_t nemId_;        // src nem
    uint64_t rx_seqnum_;    // seqnum
    timeval  rx_time_;      // actual rx time
    timeval  tx_time_;      // actual tx time
    timeval  sf_time_;      // slot time
    float peak_sum_;        // sum of power over whole message
    uint32_t num_samples_;  // number of segments in peak_sum

    RxData() {}

    RxData(uint16_t nemId,
           uint64_t rx_seqnum,
           const timeval & rx_time,
           const timeval & tx_time,
           const timeval & sf_time,
           const float & peak_sum,
           const uint32_t & num_samples) :
      nemId_(nemId),
      rx_seqnum_(rx_seqnum),
      rx_time_(rx_time),
      tx_time_(tx_time),
      sf_time_(sf_time),
      peak_sum_(peak_sum),
      num_samples_(num_samples)
    {}
  };

  struct RxControl {
    RxData rxData_;
    SINRTester SINRTester_;

    RxControl() {}

    RxControl(uint16_t nemId,
              uint64_t rx_seqnum,
              const timeval & rx_time,
              const timeval & tx_time,
              const timeval & sf_time,
              const float & peak_sum,
              const uint32_t & num_samples) :
      rxData_(nemId, rx_seqnum, rx_time, tx_time, sf_time, peak_sum, num_samples),
      SINRTester_()
    { }
  };

  typedef std::pair<Data, RxControl> RxMessage;
  typedef std::vector<RxMessage>     RxMessages;
}
}

#endif
