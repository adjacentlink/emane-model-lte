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

#include "downlinkresourcegridparams.h"


namespace {
std::map<std::uint32_t, EMANE::Models::LTE::ResourceBlockList> centerResourceBlocks_ =
  {
    {6,   { 0, 1, 2, 3, 4, 5   }},
    // 7 is not an operational bandwidth. it is used by the ue during cell search to overlap
    // with the central resource blocks carrying the PBCH when the enb is configured with
    // an odd numbered bandwidth (15, 25, 75).
    {7,   { 0, 1, 2, 3, 4, 5, 6}},
    {15,  { 4, 5, 6, 7, 8, 9,10}},
    {25,  { 9,10,11,12,13,14,15}},
    {50,  {22,23,24,25,26,27   }},
    {75,  {34,35,36,37,38,39,40}},
    {100, {47,48,49,50,51,52   }}
  };
}


EMANE::Models::LTE::DownlinkChannelRBParams::DownlinkChannelRBParams(std::uint32_t numPrbs) :
  numPrbs_{numPrbs},
  numSlots_{EMANELTE::NUM_SLOTS_PER_FRAME},
  centralPrbs_{centerResourceBlocks_[numPrbs]}
  {}


std::string
EMANE::Models::LTE::DownlinkChannelRBParams::toString()
{
  std::stringstream ss;

  for(size_t cfi=1; cfi<4; ++cfi)
    {
      ss<<"cfi="<<cfi<<std::endl;
      ss<<"------"<<std::endl;
      for(size_t row=1; row<=numPrbs_; ++row)
        {
          size_t prb = numPrbs_ - row;

          ss<<std::setw(3)<<prb<<":";

          for(size_t slot=0; slot<numSlots_; ++slot)
            {
              ss<<" "<<std::setw(2)<<params_[cfi][slot][prb].res_;
              ss<<" ("<<params_[cfi][slot][prb].first_;
              ss<<","<<params_[cfi][slot][prb].last_;
              ss<<")";

              if(slot%2)
                {
                  ss<<" ";
                }
            }
          ss<<std::endl;
        }
      ss<<std::endl;
    }

  return ss.str();
}


EMANE::Models::LTE::SlotRBParams &
EMANE::Models::LTE::DownlinkChannelRBParams::slotParams(size_t cfi, size_t slot_idx)
{
  return params_[cfi][slot_idx];
}  


EMANE::Models::LTE::PDSCHParams::PDSCHParams(std::uint32_t numPrbs, std::uint32_t numSymbols) :
  DownlinkChannelRBParams{numPrbs}
  {
    std::uint32_t defaultSlotREs{(numSymbols * 12) - 8};

    GridRBParams maxSlotREs;

    // start with maximum res in each slot
    for(size_t slot=0; slot<numSlots_; ++slot)
      {
        for(size_t prb=0; prb<numPrbs_; ++prb)
          {
            maxSlotREs[slot].push_back(ResourceBlockParams(defaultSlotREs, 0, numSymbols));
          }
      }
  
    adjustSync(maxSlotREs, numPrbs, numSymbols);
      
    adjustPBCH(maxSlotREs, numPrbs, numSymbols);

    removeControl(maxSlotREs, numPrbs);
  }

void
EMANE::Models::LTE::PDSCHParams::adjustSync(GridRBParams & maxSlotREs, std::uint32_t numPrbs, std::uint32_t numSymbols)
{
  std::uint32_t rsAdjust = (numSymbols == 7) ? 4 : 0;

  // remove res due to PSS/SSS
  for(auto prb : centralPrbs_)
    {
      std::uint32_t syncSymbols{2};

      std::uint32_t pdschSymbols{numSymbols - syncSymbols};

      ResourceBlockParams syncRBParams((pdschSymbols * 12) - rsAdjust, 0, pdschSymbols);

      maxSlotREs[0][prb] = syncRBParams;

      maxSlotREs[10][prb] = syncRBParams;
    }
    
  // add back res on the partial PSS/SSS slots when there is an odd number of RBs
  if(numPrbs % 2)
    {
      ResourceBlockParams syncRBParams{maxSlotREs[ 0][centralPrbs_[0]]};

      ResourceBlockParams syncRBParamsAdjusted{syncRBParams.res_ + 12, 0, numSymbols};

      maxSlotREs[ 0][centralPrbs_[                    0]] = syncRBParamsAdjusted;
      maxSlotREs[ 0][centralPrbs_[centralPrbs_.size()-1]] = syncRBParamsAdjusted;

      maxSlotREs[10][centralPrbs_[                    0]] = syncRBParamsAdjusted;
      maxSlotREs[10][centralPrbs_[centralPrbs_.size()-1]] = syncRBParamsAdjusted;
    }
}

void
EMANE::Models::LTE::PDSCHParams::adjustPBCH(GridRBParams & maxSlotREs, std::uint32_t numPrbs, std::uint32_t numSymbols)
{
  // remove res due to PBCH res
  for(auto prb : centralPrbs_)
    {
      std::uint32_t pbchSymbols{4};

      std::uint32_t pdschSymbols{numSymbols - pbchSymbols};

      ResourceBlockParams pbchRBParams{(pdschSymbols * 12) - 4, pbchSymbols, numSymbols};

      maxSlotREs[1][prb] = pbchRBParams;
    }

  // add back res on the partial PSS/SSS and PBCH slots when there is an odd number of RBs
  if(numPrbs % 2)
    {
      ResourceBlockParams pbchRBParams = maxSlotREs[ 1][centralPrbs_[ 0]];

      ResourceBlockParams pbchRBParamsAdjusted = ResourceBlockParams(pbchRBParams.res_ + 22, 0, numSymbols);

      maxSlotREs[ 1][centralPrbs_[                    0]] = pbchRBParamsAdjusted;
      maxSlotREs[ 1][centralPrbs_[centralPrbs_.size()-1]] = pbchRBParamsAdjusted;
    }
}

void
EMANE::Models::LTE::PDSCHParams::removeControl(GridRBParams & maxSlotREs, std::uint32_t numPrbs)
{
  // finally remove the res contained in the control region
  std::uint32_t cfiAdjust = (numPrbs == 6) ? 1 : 0;

  for(size_t cfi=1; cfi<4; ++cfi)
    {
      for(size_t slot=0; slot<numSlots_; ++slot)
        {
          if ((slot % 2) == 0)
            {
              // remove the first cfi symbols, less the rs in the control region, already accounted for
              std::uint32_t controlREs = ((slot % 2) == 0) ? (cfi + cfiAdjust) * 12 - 4 : 0;

              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  ResourceBlockParams rbParams{maxSlotREs[slot][prb]};

                  ResourceBlockParams rbParamsNew(rbParams.res_ - controlREs, cfi, rbParams.last_);

                  params_[cfi][slot].push_back(rbParamsNew);
                }
            }
          else
            {
              // no adjustment for second slot
              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  ResourceBlockParams rbParams{maxSlotREs[slot][prb]};

                  params_[cfi][slot].push_back(rbParams);
                }
            }
        }
    }
}



EMANE::Models::LTE::PBCHParams::PBCHParams(std::uint32_t numPrbs, std::uint32_t numSymbols) :
  DownlinkChannelRBParams{numPrbs}
{
  std::uint32_t defaultSlotREs{0};

  GridRBParams slot_res;

  // start with 0
  for(size_t slot=0; slot<numSlots_; ++slot)
    {
      for(size_t prb=0; prb<numPrbs; ++prb)
        {
          slot_res[slot].push_back(ResourceBlockParams(defaultSlotREs, numSymbols, numSymbols));
        }
    }

  std::uint32_t pbchSymbols{4};

  for(auto pbch_prb : centralPrbs_)
    {
      slot_res[1][pbch_prb] = ResourceBlockParams(pbchSymbols * 12 - 8, 0, pbchSymbols);
      slot_res[1][pbch_prb] = ResourceBlockParams(pbchSymbols * 12 - 8, 0, pbchSymbols);
    }

  if(centralPrbs_.size() % 2)
    {
      slot_res[1][centralPrbs_[                    0]] = ResourceBlockParams(0, numSymbols, numSymbols);
      slot_res[1][centralPrbs_[centralPrbs_.size()-1]] = ResourceBlockParams(0, numSymbols, numSymbols);
    }

  params_.insert(std::pair<uint32_t, GridRBParams>(1, slot_res));
  params_.insert(std::pair<uint32_t, GridRBParams>(2, slot_res));
  params_.insert(std::pair<uint32_t, GridRBParams>(3, slot_res));
}  



EMANE::Models::LTE::SyncParams::SyncParams(std::uint32_t numPrbs, std::uint32_t numSymbols) :
  DownlinkChannelRBParams{numPrbs}
{
  std::uint32_t defaultSlotREs{0};

  GridRBParams slot_res;

  // start with 0
  // start with 0
  for(size_t slot=0; slot<numSlots_; ++slot)
    {
      for(size_t prb=0; prb<numPrbs; ++prb)
        {
          slot_res[slot].push_back(ResourceBlockParams(defaultSlotREs, numSymbols, numSymbols));
        }
    }

  std::uint32_t syncSymbols{2};

  for(auto sync_prb : centralPrbs_)
    {
      slot_res[ 0][sync_prb] = ResourceBlockParams(syncSymbols * 12, numSymbols - syncSymbols, numSymbols);
      slot_res[10][sync_prb] = ResourceBlockParams(syncSymbols * 12, numSymbols - syncSymbols, numSymbols);
    }

  if (centralPrbs_.size() % 2)
    {
      slot_res[ 0][centralPrbs_[                    0]] = ResourceBlockParams(0, numSymbols, numSymbols);
      slot_res[ 0][centralPrbs_[centralPrbs_.size()-1]] = ResourceBlockParams(0, numSymbols, numSymbols);

      slot_res[10][centralPrbs_[                    0]] = ResourceBlockParams(0, numSymbols, numSymbols);
      slot_res[10][centralPrbs_[centralPrbs_.size()-1]] = ResourceBlockParams(0, numSymbols, numSymbols);
    }
    
  params_.insert(std::pair<uint32_t, GridRBParams>(1, slot_res));
  params_.insert(std::pair<uint32_t, GridRBParams>(2, slot_res));
  params_.insert(std::pair<uint32_t, GridRBParams>(3, slot_res));
}



EMANE::Models::LTE::ControlRegionParams::ControlRegionParams(std::uint32_t numPrbs, std::uint32_t numSymbols) :
  DownlinkChannelRBParams{numPrbs}
{
  // number of control regions symbols for 1.3 MHz bandwidth is cfi + 1
  std::uint32_t extra_symbol = (numPrbs == 6) ? 1 : 0;

  for(size_t cfi=1; cfi<4; ++cfi)
    {
      for(size_t slot=0; slot<numSlots_; ++slot)
        {
          if(slot % 2)
            {
              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  params_[cfi][slot].push_back(ResourceBlockParams(0, numSymbols, numSymbols));
                }
            }
          else
            {
              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  params_[cfi][slot].push_back(ResourceBlockParams(12*(cfi+extra_symbol) - 4, 0, (cfi+extra_symbol)));
                }
            }
        }
    }
}


EMANE::Models::LTE::PMCHParams::PMCHParams(std::uint32_t numPrbs) :
  DownlinkChannelRBParams{numPrbs}
{
  // PMCH can be configured with 7.5kHz or 15kHz carrier bandwidth. Assuming
  // 15kHz bandwidth here, same as for PDSCH.
  uint32_t numCarriersPerSymbol{12};

  uint32_t numSymbols{6};  // pmch is always extended cyclic prefix

  uint32_t pmchReferenceREsPerSymbol{6};

  // PMCH frames have 1 or 2 symbols dedicated to the non-mbsfn region. This is
  // separate configuration than cfi for pdsch subframes.
  for(size_t cfi=1; cfi<3; ++cfi)
    {
      for(size_t slot=0; slot<numSlots_; ++slot)
        {
          if(slot % 2)
            {
              // 2 symbols containing references in second pmch slot
              uint32_t slot1REs = (numSymbols * numCarriersPerSymbol) - (2 * pmchReferenceREsPerSymbol);

              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  params_[cfi][slot].push_back(ResourceBlockParams(slot1REs, 0, numSymbols));
                }
            }
          else
            {
              // 1 symbol containing references in first pmch slot
              uint32_t slot0REs = ((numSymbols-cfi) * numCarriersPerSymbol) - pmchReferenceREsPerSymbol;

              for(size_t prb=0; prb<numPrbs; ++prb)
                {
                  params_[cfi][slot].push_back(ResourceBlockParams(slot0REs, cfi, numSymbols));
                }
            }
        }
    }
}
