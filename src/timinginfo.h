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

#ifndef EMANELTE_MHAL_TIMINGINFO_H
#define EMANELTE_MHAL_TIMINGINFO_H

#include <emane/application/logger.h>
#include <sys/time.h>
#include <unistd.h>


namespace EMANELTE {
namespace MHAL {

  const time_t USEC_PER_SECOND = 1000000;

  inline time_t tvToUseconds(const timeval & tv)
  {
    return (tv.tv_sec * USEC_PER_SECOND) + tv.tv_usec;
  }

  inline float tvToSeconds(const timeval & tv)
  {
    return (tv.tv_sec + tv.tv_usec/1e6);
  }

  inline EMANE::Microseconds tvToMicroseconds(const timeval & tv)
  {
    return EMANE::Microseconds{tv.tv_sec * USEC_PER_SECOND + tv.tv_usec};
  }


  inline timeval tsToTv(const time_t ts)
  {
    return timeval{ts/USEC_PER_SECOND, ts%USEC_PER_SECOND};
  }


  inline EMANE::TimePoint tvToTp(const timeval & tv)
  {
    return EMANE::TimePoint{EMANE::Microseconds{tv.tv_sec * USEC_PER_SECOND + tv.tv_usec}};
  }

  inline uint32_t getMessageBin(const timeval & tv, std::uint64_t tsSfInterval)
  {
    return (tvToUseconds(tv) / tsSfInterval) % EMANELTE::NUM_SF_PER_FRAME;
  }


  class TimingInfo {
  public:

    TimingInfo(timeval tvSfInterval,
               timeval tvSlotInterval,
               time_t tsSfIntervalUsec) :
      tvCurrSf_(),
      tvNextSf_(),
      tvSfInterval_(tvSfInterval),
      tvSlotInterval_(tvSlotInterval),
      tsSfIntervalUsec_(tsSfIntervalUsec)
    { 
      init_mutex(&mutex_);
    }

    void set_tv_sf_interval(uint32_t tv_sec, uint32_t tv_usec)
    {
      tvSfInterval_.tv_sec  = tv_sec;
      tvSfInterval_.tv_usec = tv_usec;
    }

    void set_tv_slot_interval(uint32_t tv_sec, uint32_t tv_usec)
    {
      tvSlotInterval_.tv_sec  = tv_sec;
      tvSlotInterval_.tv_usec = tv_usec;
    }

    uint32_t tsSfIntervalUsec()
    {
      return tsSfIntervalUsec_;
    }

    void set_ts_sf_interval_usec(uint32_t tv_usec)
    {
      tsSfIntervalUsec_ = tv_usec;
    }

    // called after lock is set 
    uint32_t stepTime()
    {
      tvCurrSf_ = tvNextSf_;

      timeradd(&tvNextSf_, &tvSfInterval_, &tvNextSf_);

      // new upstream depost bin = curr_bin + 4
      return (getMessageBin(tvCurrSf_, tsSfIntervalUsec_) + 4) % EMANELTE::NUM_SF_PER_FRAME;
    }

    // called after lock is set 
    void alignTime(uint32_t nof_advance_sf = 0)
    {
      timeval tvSos;

      gettimeofday(&tvSos, NULL);

      // wait for next second boundry
      if(tvSos.tv_usec > 0)
        {
          usleep(USEC_PER_SECOND - tvSos.tv_usec);

          tvSos.tv_sec += 1;
          tvSos.tv_usec = 0;
        }

      tvCurrSf_ = tvSos;

      // set next sf time
      timeradd(&tvCurrSf_, &tvSfInterval_, &tvNextSf_);

      // advance curr and next sf times
      for(uint32_t n = 0; n < nof_advance_sf; ++n)
        {
          tvCurrSf_ = tvNextSf_;
          timeradd(&tvNextSf_, &tvSfInterval_, &tvNextSf_);

          usleep(tsSfIntervalUsec_);
        }
    }

    const timeval & getCurrSfTime() const
    {
      return tvCurrSf_;
    }

    const timeval & getNextSfTime() const
    {
      return tvNextSf_;
    }

    inline std::pair<timeval, timeval> get_slot_times(const timeval &tvSfStart, int slot)
    {
      timeval tv1, tv2;

      // first slot
      if(slot == 0)
        {
          tv1 = tvSfStart;

          timeradd(&tvSfStart, &tvSlotInterval_, &tv2);
        }
      // second slot
      else if(slot == 1)
        {
          timeradd(&tvSfStart, &tvSlotInterval_, &tv1);

          timeradd(&tvSfStart, &tvSfInterval_, &tv2);
        }
      // both
      else
        {
          tv1 = tvSfStart;

          timeradd(&tvSfStart, &tvSfInterval_, &tv2);
        }

      return std::pair<timeval, timeval>(tv1, tv2); 
    }


    // get the slot index  0 for the first slot, 1 for the second slot, 2 for both
    int get_slot_number(const EMANE::Microseconds & duration, const EMANE::Microseconds & offset)
    {
      if(tvToMicroseconds(tvSfInterval_) == duration)
        {
          return 2;
        }
      else
        {
          return offset == EMANE::Microseconds{} ? 0 : 1;
        }
    }


    inline void lockTime()
    {
      LOCK_WITH_CHECK(&mutex_);
    } 
 
    inline void unlockTime()
    {
      UNLOCK_WITH_CHECK(&mutex_);
    } 
 
  private:
    timeval tvCurrSf_;
    timeval tvNextSf_;
    timeval tvSfInterval_;
    timeval tvSlotInterval_;
    time_t  tsSfIntervalUsec_;

    pthread_mutex_t mutex_;
  };

}
}

#endif
