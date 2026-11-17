// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TIMEUTIL_H
#define G2O_TIMEUTIL_H

#include <g2o/config.h>
#include <g2o/core/abi.h>
#include <g2o/stuff/g2o_stuff_api.h>

#ifndef G2O_HAVE_SYS_TIME_H
#include <ctime>
#else
#include <sys/time.h>
#endif

#include <string>

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time related stuff
 */

/// Executes code, only if secs are gone since last exec.
/// extended version, in which the current time is given, e.g., timestamp of IPC message
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = (currentTime); \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if (s_now_ - s_lastDone_ > (secs)) { \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, g2o::get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code) \
  if(1) { \
    double _start_time_ = g2o::get_time(); \
    code; \
    fprintf(stderr, "%s took %f sec\n", text, g2o::get_time() - _start_time_); \
  } else \
    (void) 0
#endif

#include <g2o/prefix.h>

G2O_START_NAMESPACE

#ifndef G2O_HAVE_GETTIMEOFDAY
typedef struct timeval {
  long tv_sec;
  long tv_usec;
} timeval;
G2O_STUFF_API int gettimeofday(struct timeval *tv, struct timezone *tz);
#endif

/**
 * return the current time in seconds since 1. Jan 1970
 */
inline double get_time()
{
  struct timeval ts{};
  gettimeofday(&ts,nullptr);
  return ts.tv_sec + ts.tv_usec*1e-6;
}

/**
 * return a monotonic increasing time which basically does not need to
 * have a reference point. Consider this for measuring how long some
 * code fragments required to execute.
 *
 * On Linux we call clock_gettime() on other systems we currently
 * call get_time().
 */
G2O_STUFF_API double get_monotonic_time();

/**
 * \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create and instance at the beginning of the function.
 */
class G2O_STUFF_API ScopeTime {
  public:
    ScopeTime(const char* title);
    ~ScopeTime();
  private:
    std::string _title;
    double _startTime;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME \
  g2o::ScopeTime scopeTime(__PRETTY_FUNCTION__)
#endif


// @}
#endif
