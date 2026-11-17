// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

G2O_START_NAMESPACE

  EdgeSE2Segment2DPointLine::EdgeSE2Segment2DPointLine() :
    BaseBinaryEdge<3, Vector3D, VertexSE2, VertexSegment2D>()
  {
    _pointNum = 0;
  }

  bool EdgeSE2Segment2DPointLine::read(std::istream& is)
  {
    is >> _pointNum;
    for (size_t i = 0; i < 3 ; i++)
      is >> _measurement[i];
    for (size_t i = 0; i < 3 ; i++)
      for (size_t j = i; j < 3 ; j++) {
        is >> _information (i,j);
        _information (j,i) = _information (i,j);
      }
    return true;
  }

  bool EdgeSE2Segment2DPointLine::write(std::ostream& os) const
  {
    os << _pointNum << " ";
    for (size_t i = 0; i < 3 ; i++)
      os << _measurement[i] << " ";
    for (size_t i = 0; i < 3 ; i++)
      for (size_t j = i; j < 3 ; j++) {
        os << _information (i,j) << " ";
      }
    return os.good();
  }

G2O_END_NAMESPACE
