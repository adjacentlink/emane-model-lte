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

#include "uplinkresourcegridparams.h"


EMANE::Models::LTE::UplinkChannelRBParams::UplinkChannelRBParams(const ResourceBlockParams params) :
  params_{params}
{}


std::string
EMANE::Models::LTE::UplinkChannelRBParams::toString()
{
  std::stringstream ss;

  ss<<" "<<std::setw(2)<<params_.res_;
  ss<<" ("<<params_.first_;
  ss<<","<<params_.last_;
  ss<<")";

  ss<<std::endl;

  return ss.str();
}


EMANE::Models::LTE::PUSCHParams::PUSCHParams(std::uint32_t numSymbols) :
  // Demodulation reference symbol occupies 1 symbol
  UplinkChannelRBParams(ResourceBlockParams{12 * (numSymbols-1), 0, numSymbols})
{}  


EMANE::Models::LTE::PUCCHParams::PUCCHParams(std::uint32_t numSymbols) :
  // PUCCH format 1-1b get 4 symbols (normal or extended)
  // PUCCH format 2, 2a, 2b and 3 gets 5 symbols.
  // Here assuming 4, TODO, make numREs dependent on PUCCH format
  UplinkChannelRBParams(ResourceBlockParams{12 * 4, 0, numSymbols})
{}  


EMANE::Models::LTE::PRACHParams::PRACHParams(std::uint32_t numSymbols) :
  // PRACH subcarriers are spaced 1.25kHz apart and occupy a spectrum equivalent to
  // 6 contiguous resource blocks in the frequency domain. The symbols sequence is
  // 800 usec in length, but the entire PRACH time length varies depending on length
  // of the cyclic prefix and the number of times the sequence is sent. The format
  // is dictated by the PRACH configuration index with PRACH lengths ranging from
  // 1 to 3 subframes in time (formats 0 to 3). Here PRACH is being modeled as
  // format 0 only (1 subframe in length) with an equal number of subcarriers
  // in each resource block. In practice there are slightly fewer subcarriers
  // in the end 2 blocks due to 15kHZ guard bands.
  UplinkChannelRBParams(ResourceBlockParams{144, 0, numSymbols})
{}  
