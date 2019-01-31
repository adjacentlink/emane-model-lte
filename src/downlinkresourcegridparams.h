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

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <list>
#include <map>
#include <sstream>
#include <vector>
#include "ltedefs.h"
#include "commonresourcegridparams.h"
#include "libemanelte/otacommon.pb.h"


#ifndef EMANELTE_MHAL_DOWNLINKRESOURCEGRIDPARAMS_H
#define EMANELTE_MHAL_DOWNLINKRESOURCEGRIDPARAMS_H

namespace EMANE
{
namespace Models
{
namespace LTE
{

class DownlinkChannelRBParams
{
public:
  DownlinkChannelRBParams(std::uint32_t numPrbs);

  std::string toString();

  SlotRBParams & slotParams(size_t cfi, size_t slot_idx);

protected:
  std::uint32_t numPrbs_;
  std::uint32_t numSlots_;
  ResourceBlockList centralPrbs_;
  std::map<std::uint32_t, GridRBParams> params_; // resource block params by cfi
};

  
class PDSCHParams : public DownlinkChannelRBParams
{
public:
  PDSCHParams(std::uint32_t numPrbs, std::uint32_t numSymbols);

private:
  void adjustSync(GridRBParams & maxSlotREs, std::uint32_t numPrbs, std::uint32_t numSymbols);

  void adjustPBCH(GridRBParams & maxSlotREs, std::uint32_t numPrbs, std::uint32_t numSymbols);

  void removeControl(GridRBParams & maxSlotREs, std::uint32_t numPrbs);
};


class PBCHParams : public DownlinkChannelRBParams
{
public:
  PBCHParams(std::uint32_t numPrbs, std::uint32_t numSymbols);
};


class SyncParams : public DownlinkChannelRBParams
{
public:
  SyncParams(std::uint32_t numPrbs, std::uint32_t numSymbols);
};


class ControlRegionParams : public DownlinkChannelRBParams
{
public:
  ControlRegionParams(std::uint32_t numPrbs, std::uint32_t numSymbols);
};


class PMCHParams : public DownlinkChannelRBParams
{
public:
  PMCHParams(std::uint32_t numPrbs);
};


struct DownlinkResourceGridParams
{
  std::map<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams> params;

  DownlinkResourceGridParams(std::uint32_t numPrbs, std::uint32_t numSymbols)
  {
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PDSCH,   PDSCHParams{numPrbs, numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PMCH,    PMCHParams{numPrbs}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PBCH,    PBCHParams{numPrbs, numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_SSS_PSS, SyncParams{numPrbs, numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PCFICH,  ControlRegionParams{numPrbs, numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PDCCH,   ControlRegionParams{numPrbs, numSymbols}));
    params.insert(std::pair<EMANELTE::MHAL::CHANNEL_TYPE, DownlinkChannelRBParams>(EMANELTE::MHAL::CHAN_PHICH,   ControlRegionParams{numPrbs, numSymbols}));
  }
};

}
}
}

#endif

/*
int main(void)
{
  std::uint32_t numSymbols[]{6,7};

  std::uint32_t numPrbs[]{6,15,25,50,75,100};

  for(auto symbols : numSymbols)
    {
      for(auto prbs : numPrbs)
        {
          EMANELTE::MHAL::DownlinkResourceGridParams rg{prbs, symbols};

          std::cout<<"pdsch"<<std::endl;
          std::cout<<"-----"<<std::endl;
          std::cout<<rg.pdsch.toString()<<std::endl;
          std::cout<<"pbch"<<std::endl;
          std::cout<<"----"<<std::endl;
          std::cout<<rg.pbch.toString()<<std::endl;
          std::cout<<"sync"<<std::endl;
          std::cout<<"----"<<std::endl;
          std::cout<<rg.sync.toString()<<std::endl;
          std::cout<<"control"<<std::endl;
          std::cout<<"-------"<<std::endl;
          std::cout<<rg.control.toString()<<std::endl;
        }
    }
}
*/
