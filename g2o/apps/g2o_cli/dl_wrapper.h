// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DL_WRAPPER_H
#define G2O_DL_WRAPPER_H

#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include <g2o/cli/g2o_cli_api.h>
#include <g2o/core/abi.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

/**
 * \brief Loading libraries during run-time
 */
class G2O_CLI_API DlWrapper {
 public:
  DlWrapper();
  virtual ~DlWrapper();

  /**
   * open all libs from a directory matching a specific pattern.
   * @return number of loaded libs
   */
  int openLibraries(const std::string& directory,
                    const std::string& pattern = "");

  /**
   * open a specific library
   */
  bool openLibrary(const std::string& filename);

  /**
   * free all loaded libs, i.e., call dlclose()
   */
  void clear();

 protected:
#if defined(__unix__) || defined(__CYGWIN__)
  std::vector<void*> _handles;
#elif defined(_WIN32) && !defined(__CYGWIN__)
  std::vector<HMODULE> _handles;
#endif
  std::vector<std::string> _filenames;

 private:
  /**
   * it's not allowed to draw a copy of the wrapper
   */
  DlWrapper(const DlWrapper&);
  DlWrapper& operator=(const DlWrapper&);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
