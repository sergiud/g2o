// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/parameter.h>

G2O_START_NAMESPACE

  Parameter::Parameter() : _id(-1)
  {
  }

  void Parameter::setId(int id_)
  {
    _id = id_;
  }
  
G2O_END_NAMESPACE
