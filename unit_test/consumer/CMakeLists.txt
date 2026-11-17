# SPDX-FileCopyrightText: 2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

cmake_minimum_required (VERSION 3.28...4.5)
# The configuration of SuiteSparse, which g2o finds for CXSparse and CHOLMOD,
# searches for OpenMP for C and therefore needs C enabled.
project (g2o_consumer LANGUAGES C CXX)

# find_package must define no variable besides the results of g2o. The cache
# entries that the searches for its dependencies create are not variables
# of this project and are skipped. CMake restores PACKAGE_PREFIX_DIR itself
# only from version 3.30 on.
get_cmake_property (_consumer_VARIABLES_BEFORE VARIABLES)

foreach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_BEFORE)
  set ("_consumer_BEFORE_${_consumer_VARIABLE}" "${${_consumer_VARIABLE}}")
endforeach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_BEFORE)

find_package (g2o REQUIRED)

get_cmake_property (_consumer_VARIABLES_AFTER VARIABLES)
set (_consumer_LEAKED)

foreach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_AFTER)
  get_property (_consumer_IS_CACHED CACHE ${_consumer_VARIABLE} PROPERTY TYPE
    SET)

  if (_consumer_IS_CACHED
      OR _consumer_VARIABLE MATCHES "^(g2o_|CMAKE_MATCH_|_consumer_)"
      OR _consumer_VARIABLE STREQUAL "PACKAGE_PREFIX_DIR")
    continue ()
  endif ()

  if (NOT _consumer_VARIABLE IN_LIST _consumer_VARIABLES_BEFORE)
    list (APPEND _consumer_LEAKED ${_consumer_VARIABLE})
  elseif (NOT "${${_consumer_VARIABLE}}" STREQUAL
      "${_consumer_BEFORE_${_consumer_VARIABLE}}")
    list (APPEND _consumer_LEAKED ${_consumer_VARIABLE})
  endif (NOT _consumer_VARIABLE IN_LIST _consumer_VARIABLES_BEFORE)
endforeach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_AFTER)

if (_consumer_LEAKED)
  message (FATAL_ERROR "find_package (g2o) defined or changed the variables "
    "${_consumer_LEAKED}, expected only results of g2o")
endif (_consumer_LEAKED)

# The libraries a g2o build may export. A library missing from the build,
# such as a solver whose dependency was not found, is skipped.
set (_consumer_COMPONENTS
  cli
  core
  csparse_extension
  freeglut_minimal
  hierarchical
  interface
  opengl_helper
  parser
  solver_cholmod
  solver_csparse
  solver_dense
  solver_eigen
  solver_pcg
  solver_slam2d_linear
  solver_structure_only
  stuff
  types_data
  types_icp
  types_sba
  types_sclam2d
  types_sim3
  types_slam2d
  types_slam2d_addons
  types_slam3d
  types_slam3d_addons
)

add_executable (consumer consumer.cpp)

foreach (_consumer_COMPONENT IN LISTS _consumer_COMPONENTS)
  if (TARGET g2o::${_consumer_COMPONENT})
    string (TOUPPER ${_consumer_COMPONENT} _consumer_DEFINITION)
    target_compile_definitions (consumer PRIVATE
      G2O_CONSUMER_HAVE_${_consumer_DEFINITION}
    )
    target_link_libraries (consumer PRIVATE g2o::${_consumer_COMPONENT})
    message (STATUS "Consuming g2o::${_consumer_COMPONENT}")
  endif (TARGET g2o::${_consumer_COMPONENT})
endforeach (_consumer_COMPONENT IN LISTS _consumer_COMPONENTS)

enable_testing ()
add_test (NAME consumer COMMAND consumer)
