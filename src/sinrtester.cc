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


#include "libemanelte/sinrtester.h"
#include "sinrtesterimpl.h"
#include "utils.h"


EMANELTE::MHAL::SINRTester::SINRTester(const SINRTesterImpls & impls)
{
  impls_ = std::move(impls);
}

EMANELTE::MHAL::SINRTester & EMANELTE::MHAL::SINRTester::operator = (const SINRTester & rhs)
{
  if(&rhs == this)
   {
     return *this;
   }

  release();

  impls_ = std::move(rhs.impls_);

  return *this;
}


void
EMANELTE::MHAL::SINRTester::reset(const SINRTesterImpls & impls)
{
  release();

  impls_ = std::move(impls);
}



EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::SINRTester::sinrCheck2(CHANNEL_TYPE ctype, uint64_t carrierFrequencyHz) const
{
  const auto iter = impls_.find(carrierFrequencyHz);

  if(iter != impls_.end())
   {
     return iter->second->sinrCheck(ctype);
   }
  else
   {
     return EMANELTE::MHAL::SINRTester::SINRTesterResult{};
   }
}


EMANELTE::MHAL::SINRTester::SINRTesterResult
EMANELTE::MHAL::SINRTester::sinrCheck2(CHANNEL_TYPE ctype, uint16_t rnti, uint64_t carrierFrequencyHz) const
{
  const auto iter = impls_.find(carrierFrequencyHz);

  if(iter != impls_.end())
   {
     return iter->second->sinrCheck(ctype, rnti);
   }
  else
   {
     return EMANELTE::MHAL::SINRTester::SINRTesterResult{};
   }
}


void
EMANELTE::MHAL::SINRTester::release()
{
  for(auto & impl : impls_)
    {
      if(impl.second)
       {
         delete impl.second;
       }
    }

  impls_.clear();
}

