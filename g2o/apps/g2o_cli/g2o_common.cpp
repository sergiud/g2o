// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o_common.h"

#include <g2o/config.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>

#include <cstdlib>
#include <vector>

#include "dl_wrapper.h"

using namespace ::std;

/*
 * setting up the library filename patterns for the different OS
 */
#ifdef __APPLE__
#define SO_EXT "dylib"
#elif defined(_WIN32) || defined(__CYGWIN__)
#define SO_EXT "dll"
#else  // Linux
#define SO_EXT "so"
#endif

// This is used to determine where this library is
#if defined(G2O_HAVE_DLADDR)
#if (defined __unix__)
// dladdr is not available on a recent installation of Cygwin
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>
static Dl_info info;
#endif
#endif

#if defined(__unix__)
#define PATH_SEPARATOR ":"
#else  // _WIN32
#define PATH_SEPARATOR ";"

static void fakeFunctionForWindows() {}

HMODULE getMyInstance() {
  MEMORY_BASIC_INFORMATION mbi;
  if (VirtualQuery((const void*)&fakeFunctionForWindows, &mbi, sizeof(mbi))) {
    return (HMODULE)mbi.AllocationBase;
  }
  return NULL;
}
#endif

static const string TYPES_PATTERN = string("*_types_*") +
                                    string(G2O_LIBRARY_POSTFIX) + string(".") +
                                    string(SO_EXT);
static const string SOLVERS_PATTERN = string("*_solver_*") +
                                      string(G2O_LIBRARY_POSTFIX) +
                                      string(".") + string(SO_EXT);

G2O_START_NAMESPACE

void findArguments(const std::string& option, vector<string>& args, int argc,
                   char** argv) {
  args.clear();
  for (int i = 0; i < argc; ++i) {
    if (argv[i] == option && i + 1 < argc) {
      args.emplace_back(argv[i + 1]);
    }
  }
}

void loadStandardTypes(DlWrapper& dlTypesWrapper, int argc, char** argv) {
  char* envTypesPath = getenv("G2O_TYPES_DIR");
  string typesPath;

  if (envTypesPath != nullptr) {
    typesPath = envTypesPath;
  } else {
    typesPath = G2O_DEFAULT_TYPES_DIR_;
#if (defined G2O_HAVE_DLADDR)
    if (dladdr(&info, &info) != 0) {
      typesPath = getDirname(info.dli_fname);
    }
#elif (defined _WIN32)
    char libFilename[MAX_PATH + 1];
    HMODULE instance = getMyInstance();
    if (instance && GetModuleFileName(instance, libFilename, MAX_PATH) > 0) {
      typesPath = getDirname(libFilename);
    }
#endif
  }

  vector<string> paths = strSplit(typesPath, PATH_SEPARATOR);
  for (vector<string>::const_iterator it = paths.begin(); it != paths.end();
       ++it) {
    if (!it->empty()) dlTypesWrapper.openLibraries(*it, TYPES_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != nullptr) findArguments("-typeslib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end();
       ++it) {
    ngx::Log("Loading types {}", *it);
    dlTypesWrapper.openLibrary(*it);
  }
}

void loadStandardSolver(DlWrapper& dlSolverWrapper, int argc, char** argv) {
  char* envSolversPath = getenv("G2O_SOLVERS_DIR");
  string solversPath = G2O_DEFAULT_SOLVERS_DIR_;

  if (envSolversPath != nullptr) {
    solversPath = envSolversPath;
  } else {
#if (defined G2O_HAVE_DLADDR)
    if (dladdr(&info, &info) != 0) {
      solversPath = getDirname(info.dli_fname);
    }
#elif (defined _WIN32)
    char libFilename[MAX_PATH + 1];
    HMODULE instance = getMyInstance();
    if (instance && GetModuleFileName(instance, libFilename, MAX_PATH) > 0) {
      solversPath = getDirname(libFilename);
    }
#endif
  }

  vector<string> paths = strSplit(solversPath, PATH_SEPARATOR);
  for (vector<string>::const_iterator it = paths.begin(); it != paths.end();
       ++it) {
    if (!it->empty()) dlSolverWrapper.openLibraries(*it, SOLVERS_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != nullptr)
    findArguments("-solverlib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end();
       ++it) {
    ngx::Log("Loading solver {}", *it);
    dlSolverWrapper.openLibrary(*it);
  }
}

G2O_END_NAMESPACE
