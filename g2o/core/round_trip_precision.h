// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_CORE_ROUND_TRIP_PRECISION_H
#define G2O_CORE_ROUND_TRIP_PRECISION_H

#include <g2o/config.h>
#include <g2o/core/abi.h>
#include <g2o/prefix.h>

#include <ios>
#include <limits>

G2O_START_NAMESPACE

/**
 * Lets a stream write each double with as many significant digits as reading
 * it back into the same value requires, and restores the precision of the
 * stream on destruction.
 */
class RoundTripPrecision {
 public:
  explicit RoundTripPrecision(std::ios_base& stream)
      : _stream(stream),
        _precision(
            stream.precision(std::numeric_limits<double>::max_digits10)) {}

  ~RoundTripPrecision() { _stream.precision(_precision); }

  RoundTripPrecision(const RoundTripPrecision&) = delete;
  RoundTripPrecision& operator=(const RoundTripPrecision&) = delete;

 private:
  std::ios_base& _stream;
  std::streamsize _precision;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
