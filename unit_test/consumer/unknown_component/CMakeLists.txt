# SPDX-FileCopyrightText: 2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

# Requests a component that no g2o build provides, which the package must
# reject.
cmake_minimum_required (VERSION 3.28...4.5)
project (g2o_unknown_component LANGUAGES C CXX)

find_package (g2o REQUIRED COMPONENTS no_such_component)
