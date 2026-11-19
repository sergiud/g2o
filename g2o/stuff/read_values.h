// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STUFF_READ_VALUES_H
#define G2O_STUFF_READ_VALUES_H

#include <cstddef>
#include <istream>
#include <utility>
#include <vector>

namespace g2o {

/**
 * Reads the count of the values that follow it in a line. A count that is
 * negative or exceeds the maximum fails the stream.
 */
inline bool readCount(std::istream& is, std::size_t& count,
                      std::size_t maximum) {
  long long value = -1;
  if (!(is >> value) || value < 0 || std::cmp_greater(value, maximum)) {
    is.setstate(std::ios_base::failbit);
    return false;
  }
  count = static_cast<std::size_t>(value);
  return true;
}

/**
 * Appends the given number of values read from a stream. The vector grows
 * only with the values the stream holds, so that a count beyond the end of
 * the input allocates no more memory than the input itself.
 */
template <typename T>
bool readValues(std::istream& is, std::size_t count, std::vector<T>& values) {
  for (std::size_t i = 0; i < count; ++i) {
    T value{};
    if (!(is >> value)) {
      return false;
    }
    values.push_back(value);
  }
  return true;
}

}  // namespace g2o

#endif
