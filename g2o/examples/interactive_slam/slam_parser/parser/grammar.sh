#!/bin/sh
# SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
#
# SPDX-License-Identifier: BSD-2-Clause


# wrapper script to generate the parser by flex / bison

flex -o flex_scanner.cpp -i scanner.l

cp /usr/include/FlexLexer.h .

bison -o bison_parser.cpp --defines=bison_parser.h parser.yy
