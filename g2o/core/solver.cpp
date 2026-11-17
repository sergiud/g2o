// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/solver.h>
#include <g2o/core/dynamic_aligned_buffer.hpp>

#include <cstring>
#include <algorithm>

G2O_START_NAMESPACE

Solver::Solver() :
  _optimizer(nullptr), _x(nullptr), _b(nullptr), _xSize(0), _maxXSize(0),
  _isLevenberg(false), _additionalVectorSpace(0)
{
}

Solver::~Solver()
{
  free_aligned(_x);
  free_aligned(_b);
}

void Solver::resizeVector(size_t sx)
{
  size_t oldSize = _xSize;
  _xSize = sx;
  sx += _additionalVectorSpace; // allocate some additional space if requested
  if (_maxXSize < sx) {
    _maxXSize = 2*sx;
    free_aligned(_x);
    _x = allocate_aligned<double>(_maxXSize);
#ifndef NDEBUG
    memset(_x, 0, _maxXSize * sizeof(double));
#endif
    if (_b != nullptr) { // backup the former b, might still be needed for online processing
      memcpy(_x, _b, oldSize * sizeof(double));
      free_aligned(_b);
      _b = allocate_aligned<double>(_maxXSize);
      std::swap(_b, _x);
    } else {
      _b = allocate_aligned<double>(_maxXSize);
#ifndef NDEBUG
      memset(_b, 0, _maxXSize * sizeof(double));
#endif
    }
  }
}

void Solver::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

void Solver::setLevenberg(bool levenberg)
{
  _isLevenberg = levenberg;
}

void Solver::setAdditionalVectorSpace(size_t s)
{
  _additionalVectorSpace = s;
}

G2O_END_NAMESPACE
