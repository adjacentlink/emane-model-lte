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


#ifndef EMANELTE_BASICSTATISTICHELPER_H
#define EMANELTE_BASICSTATISTICHELPER_H

#include <ostatistic/service.h>
#include <ostatistic/statisticnumeric.h>
#include <ostatistic/table.h>
#include <emane/statisticnumeric.h>
#include <emane/statistictable.h>

#include <string>
#include <vector>
#include <sstream>
#include <utility>

namespace EMANELTE {
  template <class T, class R>
  void setRowWithCheck_(T * table, const R & row, const std::vector<OpenStatistic::Any> & val)
   {
     try {
       if(table)
        {
          table->setRow(row, val);
        }
     } catch (const OpenStatistic::Exception & exp) {
       fprintf(stderr, "setRow statistic table exception %s\n", exp.what());
     }
   }

  template <class T, class R>
  void addRowWithCheck_(T * table, const R & row, const std::vector<OpenStatistic::Any> & val)
   {
     try {
       if(table)
        {
          table->addRow(row, val);
        }
     } catch (const OpenStatistic::Exception & exp) {
       fprintf(stderr, "addRow statistic table exception %s\n", exp.what());
    }
  }

  template <class T, class R>
  void delRowWithCheck_(T * table, const R & row)
   {
     try {
       if(table)
        {
          table->deleteRow(row);
        }
     } catch (const OpenStatistic::Exception & exp) {
       fprintf(stderr, "delRow statistic table exception %s\n", exp.what());
    }
  }

  template <class T, class R>
  void addRowWithCheck_(T * table, const R & row, const std::vector<EMANE::Any> & val)
   {
     try {
       if(table)
        {
          table->addRow(row, val);
        }
     } catch (const EMANE::Exception & exp) {
       fprintf(stderr, "addRow statistic table exception %s\n", exp.what());
    }
  }

  template <class T, class R>
  void setRowWithCheck_(T * table, const R & row, const std::vector<EMANE::Any> & val)
   {
     try {
       if(table)
        {
          table->setRow(row, val);
        }
     } catch (const EMANE::Exception & exp) {
       fprintf(stderr, "setRow statistic table exception %s\n", exp.what());
    }
  }


 template<typename T>
 std::string toHex(T val)
  {
    std::stringstream ss;
    ss << "0x" << std::hex << val;
    return ss.str();
  }
}

#endif
