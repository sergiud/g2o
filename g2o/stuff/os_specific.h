// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OS_SPECIFIC_HH_
#define G2O_OS_SPECIFIC_HH_

#include <g2o/config.h>

#include <cstdlib>

#ifndef G2O_HAVE_VASPRINTF
#include <cstdarg>
#include <cstdio>
#endif

#ifdef G2O_HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#ifndef G2O_HAVE_DRAND48
#define drand48() ((double)rand() / (double)RAND_MAX)
#endif

#ifndef G2O_HAVE_VASPRINTF
int vasprintf(char** strp, const char* fmt, va_list ap);
#endif

#endif
