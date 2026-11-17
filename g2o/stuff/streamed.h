// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STUFF_STREAMED_H
#define G2O_STUFF_STREAMED_H

#include <format>
#include <ostream>
#include <sstream>
#include <string_view>

namespace g2o {

/**
 * Formats a value with its stream insertion operator, such as an Eigen matrix
 * or a pose, which provides no std::formatter. The value must outlive the
 * wrapper, which a temporary in the arguments of a call does.
 */
template <typename T>
class Streamed {
 public:
  explicit Streamed(const T& value) : _value(value) {}

  const T& value() const { return _value; }

 private:
  const T& _value;
};

template <typename T>
Streamed(const T&) -> Streamed<T>;

}  // namespace g2o

/**
 * Writes the text of the stream insertion operator with the specification of
 * a string, such as a width and an alignment. Only character types whose
 * streams accept the value can format it, so that checking whether a value
 * is formattable does not fail for other character types.
 */
template <typename T, typename CharT>
  requires requires(std::basic_ostream<CharT>& stream, const T& value) {
    stream << value;
  }
struct std::formatter<g2o::Streamed<T>, CharT>
    : std::formatter<std::basic_string_view<CharT>, CharT> {
  template <typename FormatContext>
  auto format(const g2o::Streamed<T>& streamed, FormatContext& context) const {
    std::basic_ostringstream<CharT> stream;
    stream << streamed.value();
    return std::formatter<std::basic_string_view<CharT>, CharT>::format(
        stream.view(), context);
  }
};

#endif
