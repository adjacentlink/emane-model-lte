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


#ifndef EMANELTE_SINRTESTER_H
#define EMANELTE_SINRTESTER_H

#include <map>
#include <memory>
#include "libemanelte/otacommon.pb.h"


namespace EMANELTE {
namespace MHAL {

class SINRTesterImpl;

// <carrier frequency, carrier id>
using SINRTesterKey = std::pair<std::uint64_t, std::uint32_t>;
 
using SINRTesterImpls = std::map<SINRTesterKey, std::shared_ptr<SINRTesterImpl>>;
 
 class SINRTester
  {
    public:
     SINRTester(const SINRTesterImpls & impls);

     SINRTester & operator = (const SINRTester & rhs);

     struct SINRTesterResult {
       const bool   bFound_;    // sinr tester was set
       const bool   bPassed_;   // sinr passed curve check
       const float  sinr_dB_;
       const float  noiseFloor_dBm_;

     SINRTesterResult() :
       bFound_{false},
       bPassed_{false},
       sinr_dB_{-201.0f},
       noiseFloor_dBm_{-201.0f}
     { }

     SINRTesterResult(const bool bPassed,
                      const float sinr,
                      const float noiseFloor) :
       bFound_{true},
       bPassed_{bPassed},
       sinr_dB_{sinr},
       noiseFloor_dBm_{noiseFloor}
     { }
    };
    
   void release();
  
   void reset(const SINRTesterImpls & impls);

   SINRTesterResult sinrCheck2(const CHANNEL_TYPE ctype,
                               const uint64_t carrrierFrequencyHz,
                               const uint32_t carrierId = 0) const;

   SINRTesterResult sinrCheck2(const CHANNEL_TYPE ctype,
                               const uint16_t rnti,
                               const uint64_t carrrierFrequencyHz,
                               const uint32_t carrierId = 0) const;

  private:
    SINRTesterImpls impls_;
 };

}
}

#endif
