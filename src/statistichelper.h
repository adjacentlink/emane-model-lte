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

#ifndef EMANELTE_MHAL_STATISTICHELPER_H
#define EMANELTE_MHAL_STATISTICHELPER_H

#include <ostatistic/service.h>
#include <ostatistic/exception.h>
#include <emane/application/logger.h>


namespace EMANELTE {
namespace MHAL {

class StatisticHelper
{
  public:

  StatisticHelper(EMANE::Application::Logger & logger) :
    logger_(logger)
  { }


  template <class T, class I>
  void setCellWithCheck(const char * func, T * tbl, const I & idx, std::uint64_t col, const OpenStatistic::Any & val)
  {
    try {
      if(tbl)
       {
         tbl->setCell(idx, col, val);
       }
      else
       {
         logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s setCell invalid statistic table", func);
       }
     } 
    catch (const OpenStatistic::Exception & exp) {
        logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s setCell statistic table exception %s", func, exp.what());
    }
  }

  template <class T, class I>
  void setRowWithCheck(const char * func, T * tbl, const I & idx, const std::vector<OpenStatistic::Any> & val)
  {
    try {
      if(tbl)
       {
         tbl->setRow(idx, val);
       }
      else
       {
         logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s setRow invalid statistic table", func);
       }
     } 
    catch (const OpenStatistic::Exception & exp) {
        logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s setRow statistic table exception %s", func, exp.what());
    }
  }

  template <class T, class I>
  void addRowWithCheck(const char * func, T * tbl, const I & idx, const std::vector<OpenStatistic::Any> & val)
  {
    try {
      if(tbl)
       {
         tbl->addRow(idx, val);
       }
      else
       {
         logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s addRow invalid statistic table", func);
       }
     } 
    catch (const OpenStatistic::Exception & exp) {
        logger_.log(EMANE::ERROR_LEVEL, "MHAL::%s addRow statistic table exception %s", func, exp.what());
    }
  }


  private:

  EMANE::Application::Logger & logger_;


};

  
}
}

#endif // EMANELTE_STATISTICMANAGER_H
