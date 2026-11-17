// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/stuff/timeutil.h>

#include <iostream>

#ifdef _WIN32
#include <windows.h>

#include <ctime>
#endif

#ifdef G2O_HAVE_UNISTD_H
#include <unistd.h>
#endif

G2O_START_NAMESPACE

#if defined(_WIN32) && !defined(G2O_HAVE_GETTIMEOFDAY)
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif

struct timezone {
  int tz_minuteswest; /* minutes W of Greenwich */
  int tz_dsttime;     /* type of dst correction */
};

int gettimeofday(struct timeval* tv, struct timezone* tz) {
  // Define a structure to receive the current Windows filetime
  FILETIME ft;

  // Initialize the present time to 0 and the timezone to UTC
  unsigned __int64 tmpres = 0;
  static int tzflag = 0;

  if (NULL != tv) {
    GetSystemTimeAsFileTime(&ft);

    // The GetSystemTimeAsFileTime returns the number of 100 nanosecond
    // intervals since Jan 1, 1601 in a structure. Copy the high bits to
    // the 64 bit tmpres, shift it left by 32 then or in the low 32 bits.
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    // Convert to microseconds by dividing by 10
    tmpres /= 10;

    // The Unix epoch starts on Jan 1 1970.  Need to subtract the difference
    // in seconds from Jan 1 1601.
    tmpres -= DELTA_EPOCH_IN_MICROSECS;

    // Finally change microseconds to seconds and place in the seconds value.
    // The modulus picks up the microseconds.
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }

  if (NULL != tz) {
    if (!tzflag) {
      _tzset();
      tzflag++;
    }

    long sec;
    int hours;
    _get_timezone(&sec);
    _get_daylight(&hours);

    // Adjust for the timezone west of Greenwich
    tz->tz_minuteswest = sec / 60;
    tz->tz_dsttime = hours;
  }

  return 0;
}
#endif

ScopeTime::ScopeTime(const char* title)
    : _title(title), _startTime(get_monotonic_time()) {}

ScopeTime::~ScopeTime() {
  std::cerr << _title << " took " << 1000 * (get_monotonic_time() - _startTime)
            << "ms.\n";
}

double get_monotonic_time() {
#if (defined(_POSIX_TIMERS) && (_POSIX_TIMERS + 0 >= 0) && \
     defined(_POSIX_MONOTONIC_CLOCK))
  struct timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
#else
  return get_time();
#endif
}

G2O_END_NAMESPACE
