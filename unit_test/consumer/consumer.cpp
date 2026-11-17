// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

// Includes a public header of every installed g2o library, spelled as the
// headers of g2o spell them, and refers to an exported symbol where the
// library has one, so that a header installed at the wrong place fails to
// compile and a symbol that is not exported fails to link.

#ifdef G2O_CONSUMER_HAVE_STUFF
#include <g2o/stuff/command_args.h>
#endif

#ifdef G2O_CONSUMER_HAVE_CORE
#include <g2o/core/sparse_optimizer.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_DENSE
#include <g2o/solvers/dense/linear_solver_dense.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_EIGEN
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_PCG
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_STRUCTURE_ONLY
#include <g2o/solvers/structure_only/structure_only_solver.h>
#endif

#ifdef G2O_CONSUMER_HAVE_CSPARSE_EXTENSION
#include <g2o/solvers/csparse/csparse_helper.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_CSPARSE
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_CHOLMOD
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#endif

#ifdef G2O_CONSUMER_HAVE_SOLVER_SLAM2D_LINEAR
#include <g2o/solvers/slam2d_linear/solver_slam2d_linear.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM2D
#include <g2o/types/slam2d/types_slam2d.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM3D
#include <g2o/types/slam3d/types_slam3d.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SBA
#include <g2o/types/sba/types_six_dof_expmap.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SIM3
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_ICP
#include <g2o/types/icp/types_icp.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SCLAM2D
#include <g2o/types/sclam2d/types_sclam2d.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_DATA
#include <g2o/types/data/robot_laser.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM2D_ADDONS
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM3D_ADDONS
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#endif

#ifdef G2O_CONSUMER_HAVE_CLI
#include <g2o/apps/g2o_cli/g2o_common.h>
#endif

#ifdef G2O_CONSUMER_HAVE_HIERARCHICAL
#include <g2o/apps/g2o_hierarchical/star.h>
#endif

#ifdef G2O_CONSUMER_HAVE_PARSER
#include "slam_parser/parser/driver.h"
#endif

#ifdef G2O_CONSUMER_HAVE_INTERFACE
#include "slam_parser/interface/parser_interface.h"
#endif

#ifdef G2O_CONSUMER_HAVE_FREEGLUT_MINIMAL
#include <EXTERNAL/freeglut/freeglut_minimal.h>
#endif

#ifdef G2O_CONSUMER_HAVE_OPENGL_HELPER
#include <g2o/stuff/opengl_primitives.h>
#endif

#include <cstdlib>
#include <iostream>

int main() {
  int unresolved = 0;

  // A function whose address is null was not resolved by the dynamic linker.
  auto check = [&unresolved](const char* name, const void* address) {
    if (address == nullptr) {
      std::cerr << name << " resolved to address " << address
                << ", expected a non-null address\n";
      ++unresolved;
    }
  };

#ifdef G2O_CONSUMER_HAVE_STUFF
  g2o::CommandArgs arguments;
  static_cast<void>(arguments);
#endif

#ifdef G2O_CONSUMER_HAVE_CORE
  g2o::SparseOptimizer optimizer;
  static_cast<void>(optimizer);
#endif

#ifdef G2O_CONSUMER_HAVE_CSPARSE_EXTENSION
  check("g2o::csparse_extension::writeCs2Octave",
        reinterpret_cast<const void*>(
            &g2o::csparse_extension::writeCs2Octave));
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM2D
  g2o::VertexSE2 vertexSE2;
  static_cast<void>(vertexSE2);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM3D
  g2o::VertexSE3 vertexSE3;
  static_cast<void>(vertexSE3);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SBA
  g2o::VertexSE3Expmap vertexSE3Expmap;
  static_cast<void>(vertexSE3Expmap);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SIM3
  g2o::VertexSim3Expmap vertexSim3Expmap;
  static_cast<void>(vertexSim3Expmap);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_ICP
  g2o::Edge_V_V_GICP edgeGICP;
  static_cast<void>(edgeGICP);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SCLAM2D
  g2o::VertexOdomDifferentialParams odometryParameters;
  static_cast<void>(odometryParameters);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_DATA
  g2o::RobotLaser robotLaser;
  static_cast<void>(robotLaser);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM2D_ADDONS
  g2o::VertexSegment2D vertexSegment2D;
  static_cast<void>(vertexSegment2D);
#endif

#ifdef G2O_CONSUMER_HAVE_TYPES_SLAM3D_ADDONS
  g2o::VertexPlane vertexPlane;
  static_cast<void>(vertexPlane);
#endif

#ifdef G2O_CONSUMER_HAVE_CLI
  check("g2o::loadStandardTypes",
        reinterpret_cast<const void*>(&g2o::loadStandardTypes));
#endif

#ifdef G2O_CONSUMER_HAVE_OPENGL_HELPER
  check("g2o::opengl::drawBox",
        reinterpret_cast<const void*>(&g2o::opengl::drawBox));
#endif

  return unresolved == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
