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

#ifndef EMANELTE_MHAL_UTILS_H
#define EMANELTE_MHAL_UTILS_H

#include <emane/application/logger.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>


namespace EMANELTE {
namespace MHAL {

extern EMANE::Application::Logger logger;

void init_mutex(pthread_mutex_t * m);

#define LOCK_WITH_CHECK(x)      lock_with_check((x), __FILE__, __func__, __LINE__)
#define UNLOCK_WITH_CHECK(x)  unlock_with_check((x), __FILE__, __func__, __LINE__)

inline void lock_with_check(pthread_mutex_t * x, const char * file, const char * func, int line)
{
  const int rc = pthread_mutex_lock(x);
  if(rc) {
    fprintf(stderr, "emane-model-lte:%s [%s-%s-%d] rc [%d], reason [%s], exit now !!!\n",
            __func__, file, func, line, rc, strerror(rc));

    sleep(1);
    exit(1);
  }
}


inline void unlock_with_check(pthread_mutex_t * x, const char * file, const char * func, int line)
{
  const int rc = pthread_mutex_unlock(x);
  if(rc) {
    fprintf(stderr, "emane-model-lte:%s [%s-%s-%d] rc [%d], reason [%s], exit now !!!\n",
            __func__, file, func, line, rc, strerror(rc));

    sleep(1);
    exit(1);
  }
}

  
}
}

#endif
