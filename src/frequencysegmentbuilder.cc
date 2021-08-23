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

#include "frequencysegmentbuilder.h"


void
EMANE::Models::LTE::FrequencySegmentBuilder::insert(FrequencySegmentKey key, float txPowerdBm)
{
  auto iter = segmentMap_.find(key);

  // insert into map only if segment is not already there or if new segment
  // has higher transmit power than existing one
  if(iter == segmentMap_.end() || txPowerdBm > iter->second)
    {
      segmentMap_.emplace(key, txPowerdBm);
    }
}


const EMANE::Models::LTE::OffsetDuration
EMANE::Models::LTE::FrequencySegmentBuilder::calcSegmentBoundary(uint32_t slot,
                                                                 uint32_t startSymb,
                                                                 uint32_t stopSymb,
                                                                 EMANE::Microseconds slotDuration) const
{
  // compute offset and duration based on symbol boundaries
  EMANE::Microseconds offset = (startSymb * slotDuration) / 7;
  EMANE::Microseconds fullDuration = (((stopSymb - startSymb) * slotDuration) / 7);

  // trim 1 phy noisebin duration from the end of the full duration to avoid twice adding
  // signal energy to the spectrum monitor where pdcch ends in the same bin that pdsch starts
  EMANE::Microseconds duration{fullDuration.count() - 20};

  // if the segment is in the second slot of the subframe, add
  // the first slot duration to the offset
  if(slot % 2)
    {
      offset += slotDuration;
    }

  return OffsetDuration{offset, duration};
}


EMANE::FrequencySegments
EMANE::Models::LTE::FrequencySegmentBuilder::build(EMANE::Microseconds slotDuration)
{
  EMANE::FrequencySegments segs;

  EMANELTE::FrequencyHz freq;
  uint32_t slot;
  uint32_t startSymb;
  uint32_t stopSymb;
  float txPowerdBm;

  EMANE::Microseconds offset;
  EMANE::Microseconds duration;
  
  for(auto iter : segmentMap_)
    {
      std::tie(freq, slot, startSymb, stopSymb) = iter.first;
      txPowerdBm = iter.second;

      std::tie(offset, duration) = calcSegmentBoundary(slot, startSymb, stopSymb, slotDuration);

#if 0
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::DEBUG_LEVEL,
                              "%s %03hu %s: "
                              "segment slot %u, freq %lu, offset %lu, duration %lu, txpowerdBm %0.1f",
                              "RadioModel",
                              id_,
                              __func__,
                              slot,
                              freq,
                              offset.count(),
                              duration.count(),
                              txPowerdBm);
#endif

      segs.push_back({freq, txPowerdBm, duration, offset});
    }

  // clear segments on build
  segmentMap_.clear();

  return segs;
}
