// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <cstdio>

namespace {

//! Announces that a program loaded this library as a plugin.
struct Marker {
  Marker() { std::fputs("LOADED " G2O_PLUGIN_MARKER "\n", stderr); }
};

const Marker marker;

}  // namespace
