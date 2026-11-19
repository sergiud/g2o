// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STUFF_INTERRUPT_H
#define G2O_STUFF_INTERRUPT_H

#include <g2o/core/abi.h>
#include <g2o/prefix.h>
#include <g2o/stuff/g2o_stuff_api.h>

#include <atomic>

G2O_START_NAMESPACE

/**
 * Handles interrupts of the program by setting the returned flag, which
 * stops an optimization that watches it. The third interrupt ends the
 * program with a failure, for an optimization that does not stop.
 */
G2O_STUFF_API std::atomic<bool>& stopOnInterrupt();

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
