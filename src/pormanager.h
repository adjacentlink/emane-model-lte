/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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
 */

#ifndef EMANELTE_MHAL_PORMANAGER_HEADER_
#define EMANELTE_MHAL_PORMANAGER_HEADER_

#include <string>
#include <map>
#include <tuple>
#include "ltedefs.h"
#include "libemanelte/otacommon.pb.h"

namespace EMANE
{
namespace Models
{
  namespace LTE
    {
      /**
       * @class PORManager
       *
       * @brief POR Manager responsible for loading PCR curves from
       * file and determining POR.
       *
       * PCR curves are defined per data rate, with the first curve
       * defined also serving as the default curve used when a POR is
       * requested for an undefined data rate.
       */
      class PORManager
      {
      public:
        PORManager();

        void load(const std::string & sPCRFileName);

        float getDedicatedChannelPOR(EMANELTE::MHAL::MOD_TYPE modulationType,
                                     float fSINR);

        using CurveDump  = std::map<float,float>;
        using CurveDumps = std::map<EMANELTE::MHAL::MOD_TYPE,CurveDump>;

        CurveDumps dump();

      private:
        using Curve = std::map<std::int32_t,float>; // sinr, por

        using DedicatedChannelTable = std::map<EMANELTE::MHAL::MOD_TYPE, // modulation type
                                       std::tuple<std::int32_t,  // min SINR
                                                  std::int32_t,  // max SINR
                                                  Curve>>;
        DedicatedChannelTable dedicatedChannelTable_;
      };
   }
}
}

#endif // EMANELTE_MHAL_PORMANAGER_HEADER_
