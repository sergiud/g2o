// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_DL_WRAPPER_H
#define G2O_DL_WRAPPER_H

#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include <g2o/cli/g2o_cli_api.h>
#include <g2o/core/abi.h>

G2O_START_NAMESPACE

  /**
   * \brief Loading libraries during run-time
   */
  class G2O_CLI_API DlWrapper
  {
    public:
      DlWrapper();
      virtual ~DlWrapper();

      /**
       * open all libs from a directory matching a specific pattern.
       * @return number of loaded libs
       */
      int openLibraries(const std::string& directory, const std::string& pattern = "");

      /**
       * open a specific library
       */
      bool openLibrary(const std::string& filename);

      /**
       * free all loaded libs, i.e., call dlclose()
       */
      void clear();

    protected:
# if defined (__unix__) || defined(__CYGWIN__)
      std::vector<void*> _handles;
#     elif defined (_WIN32) && !defined(__CYGWIN__)
      std::vector<HMODULE> _handles;
#     endif
      std::vector<std::string> _filenames;

    private:
      /**
       * it's not allowed to draw a copy of the wrapper
       */
      DlWrapper(const DlWrapper& );
      DlWrapper& operator=(const DlWrapper& );
  };

G2O_END_NAMESPACE

#endif
