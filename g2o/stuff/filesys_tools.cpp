// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

/***************************************************************************
 *            filesysTools.cpp
 *
 *  Fr 02 Mär 2007 23:14:08 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/
#include <g2o/stuff/filesys_tools.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string_view>
#include <system_error>

using namespace ::std;

G2O_START_NAMESPACE

namespace {

/**
 * Whether a name matches a pattern in which '*' stands for any sequence of
 * characters and '?' for any single one.
 */
bool matchesWildcard(std::string_view name, std::string_view pattern) {
  std::size_t n = 0;
  std::size_t p = 0;
  // The position after the last '*' and the name position it matched up to.
  std::size_t starPattern = std::string_view::npos;
  std::size_t starName = 0;
  while (n < name.size()) {
    if (p < pattern.size() && (pattern[p] == '?' || pattern[p] == name[n])) {
      ++n;
      ++p;
    } else if (p < pattern.size() && pattern[p] == '*') {
      starPattern = ++p;
      starName = n;
    } else if (starPattern != std::string_view::npos) {
      // The last '*' absorbs one more character.
      p = starPattern;
      n = ++starName;
    } else {
      return false;
    }
  }
  while (p < pattern.size() && pattern[p] == '*') {
    ++p;
  }
  return p == pattern.size();
}

/**
 * Position of the dot that starts the extension of a file, which a dot in the
 * name of a directory does not.
 */
std::string::size_type extensionDot(const std::string& filename) {
#ifdef _WIN32
  constexpr const char* Separators = "/\\";
#else
  constexpr const char* Separators = "/";
#endif
  const std::string::size_type lastDot = filename.find_last_of('.');
  const std::string::size_type lastSeparator =
      filename.find_last_of(Separators);
  if (lastSeparator != std::string::npos && lastDot != std::string::npos &&
      lastDot < lastSeparator) {
    return std::string::npos;
  }
  return lastDot;
}

}  // namespace

std::string getFileExtension(const std::string& filename) {
  std::string::size_type lastDot = extensionDot(filename);
  if (lastDot != std::string::npos)
    return filename.substr(lastDot + 1);
  else
    return "";
}

std::string getPureFilename(const std::string& filename) {
  std::string::size_type lastDot = extensionDot(filename);
  if (lastDot != std::string::npos)
    return filename.substr(0, lastDot);
  else
    return filename;
}

std::string getBasename(const std::string& filename) {
#ifdef _WIN32
  std::string::size_type lastSlash = filename.find_last_of('\\');
#else
  std::string::size_type lastSlash = filename.find_last_of('/');
#endif
  if (lastSlash != std::string::npos)
    return filename.substr(lastSlash + 1);
  else
    return filename;
}

std::string getDirname(const std::string& filename) {
#ifdef _WIN32
  std::string::size_type lastSlash = filename.find_last_of('\\');
#else
  std::string::size_type lastSlash = filename.find_last_of('/');
#endif
  if (lastSlash != std::string::npos)
    return filename.substr(0, lastSlash);
  else
    return "";
}

std::string changeFileExtension(const std::string& filename,
                                const std::string& newExt, bool stripDot) {
  std::string::size_type lastDot = extensionDot(filename);
  if (lastDot != std::string::npos) {
    if (stripDot)
      return filename.substr(0, lastDot) + newExt;
    else
      return filename.substr(0, lastDot + 1) + newExt;
  } else
    return filename;
}

bool fileExists(const char* filename) {
  struct stat statInfo{};
  return (stat(filename, &statInfo) == 0);
}

std::vector<std::string> getFilesByPattern(const char* pattern) {
  const std::filesystem::path path{pattern};
  const std::filesystem::path directory = path.parent_path();
  const std::string filePattern = path.filename().string();

  std::vector<std::string> result;
  std::error_code error;
  std::filesystem::directory_iterator it{
      directory.empty() ? std::filesystem::path{"."} : directory, error};
  for (; !error && it != std::filesystem::directory_iterator{};
       it.increment(error)) {
    const std::filesystem::path name = it->path().filename();
    if (matchesWildcard(name.string(), filePattern)) {
      result.push_back((directory / name).string());
    }
  }
  std::ranges::sort(result);
  return result;
}

G2O_END_NAMESPACE
