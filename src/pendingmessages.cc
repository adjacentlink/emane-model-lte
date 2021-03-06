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


#include "pendingmessages.h"



void EMANELTE::MHAL::PendingMessageBin::clear()
{
  pending_.clear();
  binTime_ = {0,0};
}


size_t EMANELTE::MHAL::PendingMessageBin::clearAndCheck()
{
  size_t size(pending_.size());

  if(size)
    {
      clear();
    }

  return size;
}


// purge and msgs that have expired.
void EMANELTE::MHAL::PendingMessageBin::add(const timeval & binTime,
                uint32_t bin,
                const PendingMessage & msg,
                StatisticManager & statisticManager)
{
  if(pending_.empty())
    {
      binTime_ = binTime;
    }
  else
    {
      if(timercmp(&binTime, &binTime_, !=))
        {
          statisticManager.updateExpiredMessages(bin, pending_.size());
          pending_.clear();
          binTime_ = binTime;
        }
    }

  pending_.push_back(msg);
}


// a msg is composed of multiple segments each with a unique frequency
// find the min sor and max eor for each frequency
// this will be used to consult the spectrum monitor later
EMANELTE::MHAL::SegmentSpans
EMANELTE::MHAL::PendingMessageBin::getSegmentSpans()
{
  SegmentSpans segmentSpans;

  for(auto msg_iter = pending_.begin(); msg_iter != pending_.end(); ++msg_iter)
    {
      const auto & otaInfo = std::get<2>(*msg_iter);

      for(auto & segment : otaInfo.segments_)
        {
          const auto frequencyHz = segment.getFrequencyHz();

          // sor = sot + offset + propagation delay
          //const auto sor = otaInfo.sot_ + segment.getOffset() + otaInfo.propDelay_;
          const auto sor = otaInfo.sot_ + segment.getOffset();
          const auto eor = sor + segment.getDuration();

          auto span = segmentSpans.find(frequencyHz);

          if(span != segmentSpans.end())
            {
              auto & min        = std::get<0>(span->second);
              auto & max        = std::get<1>(span->second);
              auto & numEntries = std::get<2>(span->second);

              min = std::min(min, sor);
              max = std::max(max, eor);
              numEntries += 1;
            }
          else
            {
              segmentSpans.insert(std::make_pair(frequencyHz, SegmentTimeSpan{sor, eor, 1}));
            }
        }
    }

  return segmentSpans;
}


EMANELTE::MHAL::PendingMessages & EMANELTE::MHAL::PendingMessageBin::get()
{
  return pending_;
}


void EMANELTE::MHAL::PendingMessageBin::unlockBin()
{
  UNLOCK_WITH_CHECK(&mutex_);
}


void EMANELTE::MHAL::PendingMessageBin::lockBin()
{
  LOCK_WITH_CHECK(&mutex_);
}
