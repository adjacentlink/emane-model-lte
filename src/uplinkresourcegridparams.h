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

#include <iostream>
#include <iomanip>
#include <sstream>
#include "ltedefs.h"
#include "libemanelte/otacommon.pb.h"
#include "commonresourcegridparams.h"


#ifndef EMANELTE_MHAL_UPLINKRESOURCEGRIDPARAMS_H
#define EMANELTE_MHAL_UPLINKRESOURCEGRIDPARAMS_H

namespace EMANE
{
namespace Models
{
namespace LTE
{

class UplinkChannelRBParams
{
public:
  UplinkChannelRBParams(const ResourceBlockParams params);

  std::string toString();

  inline const ResourceBlockParams & rbParams()
  {
    return params_;
  }

protected:
  const ResourceBlockParams params_;
};


class PUSCHParams : public UplinkChannelRBParams
{
public:
  PUSCHParams(std::uint32_t numSymbols);
};


class PUCCHParams : public UplinkChannelRBParams
{
public:
  PUCCHParams(std::uint32_t numSymbols);
};


class PRACHParams : public UplinkChannelRBParams
{
public:
  PRACHParams(std::uint32_t numSymbols);
};



struct UplinkResourceGridParams
{
  std::map<EMANELTE::MHAL::CHANNEL_TYPE, UplinkChannelRBParams> params;

  UplinkResourceGridParams(std::uint32_t numSymbols)
  {
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, UplinkChannelRBParams>(EMANELTE::MHAL::CHAN_PUSCH, PUSCHParams{numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, UplinkChannelRBParams>(EMANELTE::MHAL::CHAN_PUCCH, PUCCHParams{numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, UplinkChannelRBParams>(EMANELTE::MHAL::CHAN_PRACH, PRACHParams{numSymbols}));
  }
};

}
}
}

#endif
