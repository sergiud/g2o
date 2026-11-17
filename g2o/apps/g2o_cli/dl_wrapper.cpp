// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "dl_wrapper.h"

#include <g2o/config.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/macros.h>
#include <sys/types.h>

#include <algorithm>
#include <cstdio>
#include <iostream>

#if defined(G2O_HAVE_DLFCN_H)
#include <dlfcn.h>
#endif

#ifdef __APPLE__
#define SO_EXT "dylib"
#define SO_EXT_LEN 5
#elif defined(_WIN32) || defined(__CYGWIN__)
#define SO_EXT "dll"
#define SO_EXT_LEN 3
#else  // Linux
#define SO_EXT "so"
#define SO_EXT_LEN 2
#endif

using namespace std;

G2O_START_NAMESPACE

DlWrapper::DlWrapper() {}

DlWrapper::~DlWrapper() {
  // clear();
}

int DlWrapper::openLibraries(const std::string& directory,
                             const std::string& pattern) {
  // cerr << "# loading libraries from " << directory << "\t pattern: " <<
  // pattern << endl;
  string searchPattern = directory + "/" + pattern;
  if (pattern == "") searchPattern = directory + "/*";
  vector<string> matchingFiles = getFilesByPattern(searchPattern.c_str());

  int numLibs = 0;
  for (auto& filename : matchingFiles) {
    if (find(_filenames.begin(), _filenames.end(), filename) !=
        _filenames.end())
      continue;

    // If we are doing a release build, the wildcards will pick up the
    // suffixes; unfortunately the "_rd" extension means that we
    // don't seem to be able to filter out the incompatible files using a
    // wildcard expansion to wordexp.

#ifndef G2O_LIBRARY_POSTFIX
    if ((filename.rfind(string("_d.") + SO_EXT) ==
         filename.length() - 3 - SO_EXT_LEN) ||
        (filename.rfind(string("_rd.") + SO_EXT) ==
         filename.length() - 4 - SO_EXT_LEN) ||
        (filename.rfind(string("_s.") + SO_EXT) ==
         filename.length() - 3 - SO_EXT_LEN))
      continue;
#endif

    // open the lib
    // cerr << "loading " << filename << endl;
    if (openLibrary(filename)) numLibs++;
  }

  return numLibs;
}

void DlWrapper::clear() {
#if defined(G2O_HAVE_DLFCN_H)
  for (auto& _handle : _handles) {
    dlclose(_handle);
  }
#elif defined(_WIN32)
  for (size_t i = 0; i < _handles.size(); ++i) {
    FreeLibrary(_handles[i]);
  }
#endif
  _filenames.clear();
  _handles.clear();
}

bool DlWrapper::openLibrary(const std::string& filename) {
#if defined(G2O_HAVE_DLFCN_H)
  void* handle = dlopen(filename.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    cerr << __PRETTY_FUNCTION__ << " Cannot open library: " << dlerror()
         << '\n';
    return false;
  }
#elif defined(_WIN32)
  HMODULE handle = LoadLibrary(filename.c_str());
  if (!handle) {
    cerr << __PRETTY_FUNCTION__ << " Cannot open library." << endl;
    return false;
  }
#endif

  // cerr << "loaded " << filename << endl;

  _filenames.push_back(filename);
  _handles.push_back(handle);
  return true;
}

G2O_END_NAMESPACE
