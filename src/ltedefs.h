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

#ifndef EMANELTE_LTEDEFS_H
#define EMANELTE_LTEDEFS_H

#include <map>
#include <cstdint>
#include <sys/time.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include "emane/types.h"

namespace EMANELTE {

using FrequencyHz = std::uint64_t;

using BandwidthHz = std::uint64_t;

using FrequencyResourceBlockMap = std::map<EMANELTE::FrequencyHz, std::uint32_t>;

const int ResourceElementsPerBlock{12};

const BandwidthHz ResourceElementBandwidthHz{15000};

const BandwidthHz ResourceBlockBandwidthHz{ResourceElementBandwidthHz * ResourceElementsPerBlock};

const BandwidthHz HalfResourceBlockBandwidthHz{ResourceBlockBandwidthHz / 2};

const size_t MAX_RESOURCE_BLOCKS_PER_SF = 100;

const size_t NUM_SF_PER_FRAME = 10;

const size_t NUM_SLOTS_PER_SF = 2;

const size_t NUM_SLOTS_PER_FRAME = NUM_SF_PER_FRAME * NUM_SLOTS_PER_SF;

const timeval tv_zero_ = {0,0};


// tx timestamp adjust knob for the ota tx_timestamp, NOT the msg tti_time
const int TX_TIME_ADJUST = 0;

inline double DB_TO_MW(double dbm)
 {
   return pow(10, dbm/10.0);
 }

inline double MW_TO_DB(double mw)
 {
   return 10.0 * log10(mw);
 }
}

#endif // LTEDEFS_H
