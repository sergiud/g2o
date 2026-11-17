#!/bin/sh
# SPDX-FileCopyrightText: 2014 Rainer Kuemmerle, Giorgio Grisetti
# SPDX-FileCopyrightText: 2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause


set -ev

sudo apt-get update -qq
sudo apt-get install -qq qt6-base-dev libqglviewer-dev-qt6 libeigen3-dev libsuitesparse-dev
