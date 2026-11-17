#!/bin/bash
# SPDX-FileCopyrightText: 2014 Rainer Kuemmerle, Giorgio Grisetti
#
# SPDX-License-Identifier: BSD-2-Clause


set -ev

brew update

if brew outdated | grep -qx cmake; then
  brew upgrade cmake;
fi

brew install eigen
