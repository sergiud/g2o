# SPDX-FileCopyrightText: 2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

# A project that builds g2o as part of its own tree and enables testing for
# itself.
cmake_minimum_required (VERSION 3.28)
project (parent LANGUAGES C CXX)

include (CTest)

add_subdirectory (${G2O_SOURCE_DIR} g2o)
