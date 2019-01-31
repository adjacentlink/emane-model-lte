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



#ifndef EMANELTE_IPTRAFFICSTATS_H
#define EMANELTE_IPTRAFFICSTATS_H

#include "basicstatistichelper.h"
#include <map>

namespace EMANELTE {

  using IPTrafficTable = OpenStatistic::Table<std::string>;

  struct IPTrafficEntry {
   size_t numBytes_;
   size_t count_;

   IPTrafficEntry(size_t numBytes, size_t count = 1) :
    numBytes_{numBytes},
    count_{count}
   { }
  };

  using IPTrafficKey = std::tuple<std::uint32_t, std::uint32_t>;
  using IPTrafficDB  = std::map<IPTrafficKey, IPTrafficEntry>;

  inline std::pair<std::string, std::string> keyToString(const IPTrafficKey & key)
   {
     const std::string s1 = inet_ntoa(*(in_addr*)&(std::get<0>(key)));
     const std::string s2 = inet_ntoa(*(in_addr*)&(std::get<1>(key)));
     return std::pair<std::string, std::string>(s1, s2);
   }
 

  inline void updateIPTrafficTable_(IPTrafficTable * table,
                                    IPTrafficDB & db,
                                    std::uint32_t src,
                                    std::uint32_t dst,
                                    size_t numBytes)
  {
    const IPTrafficKey key{src, dst};

    const auto keyAsString = keyToString(key);

    auto iter = db.find(key);

    if(iter == db.end())
      {
        // new entry
        db.insert(std::make_pair(key, IPTrafficEntry{numBytes}));

        addRowWithCheck_(table, 
                         keyAsString.first + "." + keyAsString.second,
                         {OpenStatistic::Any{keyAsString.first},   // 0 src
                          OpenStatistic::Any{keyAsString.second},  // 1 dst
                          OpenStatistic::Any{std::uint64_t{1}},    // 2 count 
                          OpenStatistic::Any{numBytes},            // 3 num bytes
                          OpenStatistic::Any{time(NULL)}});        // 4 Time
      }
    else
      {
        // update entry
        iter->second.count_ += 1;
        iter->second.numBytes_ += numBytes;

        setRowWithCheck_(table, 
                         keyAsString.first + "." + keyAsString.second,
                         {OpenStatistic::Any{keyAsString.first},      // 0 src
                          OpenStatistic::Any{keyAsString.second},     // 1 dst
                          OpenStatistic::Any{iter->second.count_},    // 2 count 
                          OpenStatistic::Any{iter->second.numBytes_}, // 3 num bytes
                          OpenStatistic::Any{time(NULL)}});           // 4 Time
      }
   }
}

#endif
