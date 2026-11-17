// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

/***************************************************************************
 *            filesysTools.h
 *
 *  Fr 02 Mär 2007 23:14:21 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/

#ifndef G2O_FILESYS_TOOLS_H
#define G2O_FILESYS_TOOLS_H

#include <g2o/stuff/g2o_stuff_api.h>

/** @addtogroup utils **/
// @{

/** \file filesysTools.h
 * \brief utility functions for handling files, directory on Linux/Unix
 */

#include <g2o/prefix.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/string_tools.h>

G2O_START_NAMESPACE

/**
 * get filename extension (the part after the last .), e.g.
 * the extension of file.txt is txt
 */
G2O_STUFF_API std::string getFileExtension(const std::string& filename);

/**
 * get the filename without the extension.
 * file.txt -> file
 */
G2O_STUFF_API std::string getPureFilename(const std::string& filename);

/**
 * change the fileextension of a given filename.
 * Only if filename contains an extension, otherwise filename is returned.
 */
G2O_STUFF_API std::string changeFileExtension(const std::string& filename,
                                              const std::string& newExt,
                                              bool stripDot = false);

/**
 * return the basename of the given filename
 * /etc/fstab -> fstab
 */
G2O_STUFF_API std::string getBasename(const std::string& filename);

/**
 * return the directory of a given filename
 * /etc/fstab -> /etc
 */
G2O_STUFF_API std::string getDirname(const std::string& filename);

/**
 * check if file exists (note a directory is also a file)
 */
G2O_STUFF_API bool fileExists(const char* filename);

/**
 * return all files that match a given pattern, e.g., ~/blaa*.txt, /tmp/a*
 */
G2O_STUFF_API std::vector<std::string> getFilesByPattern(const char* pattern);

G2O_END_NAMESPACE

#include <g2o/suffix.h>

// @}
#endif
