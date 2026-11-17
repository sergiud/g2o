// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d_addons/edge_se2_segment2d_line.h>

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

  EdgeSE2Segment2DLine::EdgeSE2Segment2DLine() :
    BaseBinaryEdge<2, Vector2D, VertexSE2, VertexSegment2D>()
  {
  }

  bool EdgeSE2Segment2DLine::read(std::istream& is)
  {
    for (size_t i = 0; i < 2 ; i++)
      is >> _measurement[i];
    for (size_t i = 0; i < 2 ; i++)
      for (size_t j = i; j < 2 ; j++) {
        is >> _information (i,j);
        _information (j,i) = _information (i,j);
      }
    return true;
  }

  bool EdgeSE2Segment2DLine::write(std::ostream& os) const
  {
    for (size_t i = 0; i < 2 ; i++)
      os << _measurement[i] << " ";
    for (size_t i = 0; i < 2 ; i++)
      for (size_t j = i; j < 2 ; j++) {
        os << _information (i,j) << " ";
      }
    return os.good();
  }

G2O_END_NAMESPACE
