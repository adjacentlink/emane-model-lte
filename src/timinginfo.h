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

  inline uint32_t getMessageBin(const timeval & tv, std::uint64_t ts_sf_interval)
  {
    return (tvToUseconds(tv) / ts_sf_interval) % EMANELTE::NUM_SF_PER_FRAME;
  }


  class TimingInfo {
  public:

    TimingInfo(timeval tv_sf_interval,
               timeval tv_slot_interval,
               time_t ts_sf_interval_usec) :
      tv_curr_sf_(),
      tv_next_sf_(),
      tv_sf_interval_(tv_sf_interval),
      tv_slot_interval_(tv_slot_interval),
      ts_sf_interval_usec_(ts_sf_interval_usec)
    { 
      init_mutex(&mutex_);
    }

    void set_tv_sf_interval(uint32_t tv_sec, uint32_t tv_usec)
    {
      tv_sf_interval_.tv_sec  = tv_sec;
      tv_sf_interval_.tv_usec = tv_usec;
    }

    void set_tv_slot_interval(uint32_t tv_sec, uint32_t tv_usec)
    {
      tv_slot_interval_.tv_sec  = tv_sec;
      tv_slot_interval_.tv_usec = tv_usec;
    }

    uint32_t ts_sf_interval_usec()
    {
      return ts_sf_interval_usec_;
    }

    void set_ts_sf_interval_usec(uint32_t tv_usec)
    {
      ts_sf_interval_usec_ = tv_usec;
    }

    // called after lock is set 
    uint32_t stepTime()
    {
      tv_curr_sf_ = tv_next_sf_;

      timeradd(&tv_next_sf_, &tv_sf_interval_, &tv_next_sf_);

      // new upstream depost bin = curr_bin + 4
      return (getMessageBin(tv_curr_sf_, ts_sf_interval_usec_) + 4) % EMANELTE::NUM_SF_PER_FRAME;
    }

    // called after lock is set 
    void alignTime(uint32_t nof_advance_sf)
    {
      timeval tv_sos;

      gettimeofday(&tv_sos, NULL);

      // get next second boundry
      if(tv_sos.tv_usec > 0)
        {
          usleep(USEC_PER_SECOND - tv_sos.tv_usec);

          tv_sos.tv_sec += 1;
          tv_sos.tv_usec = 0;
        }

      tv_curr_sf_ = tv_sos;

      timeradd(&tv_sos, &tv_sf_interval_, &tv_next_sf_);

      for(uint32_t n = 0; n < nof_advance_sf; ++n)
        {
          tv_curr_sf_ = tv_next_sf_;
          timeradd(&tv_next_sf_, &tv_sf_interval_, &tv_next_sf_);

          usleep(ts_sf_interval_usec_);
        }
    }

    const timeval & getCurrSfTime() const
    {
      return tv_curr_sf_;
    }

    const timeval & getNextSfTime() const
    {
      return tv_next_sf_;
    }

    inline std::pair<timeval, timeval> get_slot_times(const timeval &tv_sf_start, int slot)
    {
      timeval tv1, tv2;

      // first slot
      if(slot == 0)
        {
          tv1 = tv_sf_start;

          timeradd(&tv_sf_start, &tv_slot_interval_, &tv2);
        }
      // second slot
      else if(slot == 1)
        {
          timeradd(&tv_sf_start, &tv_slot_interval_, &tv1);

          timeradd(&tv_sf_start, &tv_sf_interval_, &tv2);
        }
      // both
      else
        {
          tv1 = tv_sf_start;

          timeradd(&tv_sf_start, &tv_sf_interval_, &tv2);
        }

      return std::pair<timeval, timeval>(tv1, tv2); 
    }


    // get the slot index  0 for the first slot, 1 for the second slot, 2 for both
    int get_slot_number(const EMANE::Microseconds & duration, const EMANE::Microseconds & offset)
    {
      if(tvToMicroseconds(tv_sf_interval_) == duration)
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
    struct timeval tv_curr_sf_, tv_next_sf_;
    timeval tv_sf_interval_;
    timeval tv_slot_interval_;
    time_t ts_sf_interval_usec_;

    pthread_mutex_t mutex_;
  };

}
}

#endif
