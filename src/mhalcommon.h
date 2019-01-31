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

#ifndef EMANELTE_MHALCOMMON_H
#define EMANELTE_MHALCOMMON_H

#include "libemanelte/mhalconfig.h"
#include "mhalphy.h"
#include "radiomodel.h"
#include "pendingmessages.h"
#include "readymessages.h"
#include "timinginfo.h"
#include "stateinfo.h"
#include "statisticmanager.h"
#include "utils.h"

#include <emane/application/logger.h>
#include <emane/application/nembuilder.h>
#include <emane/utils/parameterconvert.h>
#include <emane/utils/spectrumwindowutils.h>
#include <emane/spectrumserviceprovider.h>
#include <emane/exception.h>


namespace EMANELTE {
namespace MHAL {



class MHALCommon : public EMANELTE::MHAL::PHY::MHALPHY
{
public:
  MHALCommon() :
    logger_{logger},
    statisticManager_(logger),
    timing_({1,0},
            {0,500000},
            USEC_PER_SECOND),
    state_{}
  {}

  void set_tti(uint16_t curr_tti);

  void set_logger(const mhal_config_t & mhal_config);

  void set_thread_priority(pthread_t tid, int policy, int priority);

  void send_msg(const Data & data, 
                TxControlMessage & txControl);

  virtual long long unsigned int get_tx_prb_frequency(int prb_index) = 0;

  void stop();

protected:
  EMANE::Application::Logger logger_;
  StatisticManager statisticManager_;
  // rx message bins, 1 for each subframe
  PendingMessageBin pendingMessageBins_[EMANELTE::NUM_SF_PER_FRAME];
  ReadyMessages readyMessageBins_[EMANELTE::NUM_SF_PER_FRAME];
  TimingInfo timing_;
  StateInfo state_;
  
  void initialize(uint32_t sf_interval_msec, const mhal_config_t & mhal_config);

  void start(uint32_t nof_advance_sf);

  void handle_upstream_msg(const Data & data,
                           const RxData & rxData,
                           const PHY::OTAInfo & otaInfo,
                           const TxControlMessage & txControl);

  void noise_worker(const uint32_t bin, const timeval & tv_sf_start);

  void clearReadyMessages(const uint32_t bin);

  virtual void init_emane() = 0;

  virtual void send_downstream(const Data & data,
                               TxControlMessage & control,
                               const EMANE::TimePoint & timestamp) = 0;

  virtual EMANE::SpectrumWindow get_noise(FrequencyHz frequency,
                                          const EMANE::Microseconds & span,
                                          const EMANE::TimePoint & sor) = 0;

  virtual void noise_processor(const uint32_t bin,
                               const EMANE::Models::LTE::SpectrumWindowCache & spectrumWindowCache) = 0;
};

}
}

#endif
