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


#ifndef EMANELTE_MHAL_PHY_H
#define EMANELTE_MHAL_PHY_H

#include "libemanelte/mhal.h"
#include "libemanelte/txcontrolmessage.pb.h"
#include "emane/types.h"
#include "emane/controls/antennareceiveinfo.h"

// radio model to lte phy interface (upstream)
namespace EMANELTE {
namespace MHAL {
namespace PHY {
  struct OTAInfo  {
    EMANE::TimePoint                     sot_;           // sot tx sf_time
    EMANE::Microseconds                  propDelay_;     // prop delay
    EMANE::Controls::AntennaReceiveInfos antennaInfos_;  // antenna recevie infos

    OTAInfo(const EMANE::TimePoint & sot,
            const EMANE::Microseconds & propDelay,
            const EMANE::Controls::AntennaReceiveInfos & antennaInfos):
      sot_{sot},
      propDelay_{propDelay},
      antennaInfos_{std::move(antennaInfos)}
    { }
  };

  class MHALPHY
  {
  public:
    virtual void handle_upstream_msg(const EMANELTE::MHAL::Data & data, 
                                     const EMANELTE::MHAL::RxControl & rxControl,
                                     const OTAInfo & otaInfo,
                                     const EMANELTE::MHAL::TxControlMessage & txControl) = 0;
  };
}
}
}


#endif
