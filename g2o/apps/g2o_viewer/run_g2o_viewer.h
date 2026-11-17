// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/abi.h>
#include <g2o/prefix.h>
#include <g2o/viewer/g2o_viewer_api.h>

G2O_START_NAMESPACE

class CommandArgs;

/**
 * \brief wrapper for running the g2o viewer
 */
class G2O_VIEWER_API RunG2OViewer {
 public:
  static int run(int argc, char** argv, CommandArgs& arg);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>
