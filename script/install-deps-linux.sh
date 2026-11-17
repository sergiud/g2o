#!/bin/sh

set -ev

sudo apt-get update -qq
sudo apt-get install -qq qt6-base-dev libqglviewer-dev-qt6 libeigen3-dev libsuitesparse-dev
