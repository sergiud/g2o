// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DRAW_HELPERS_H
#define G2O_DRAW_HELPERS_H

#include <g2o/core/abi.h>
#include <g2o/stuff/opengl_wrapper.h>

G2O_START_NAMESPACE

/**
 * draw a disk
 */
void drawDisk(GLfloat radius);

/**
 * draw a circle using GL_LINES
 */
void drawCircle(GLfloat radius, int segments = 32);

G2O_END_NAMESPACE

#endif
