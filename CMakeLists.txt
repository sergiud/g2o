# SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
# SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

cmake_minimum_required (VERSION 3.28...4.5)

# CMake 4.3 and 4.4 generate software bills of materials experimentally,
# each version behind a key of its own that must be set before the project.
# From version 4.5 on, the feature is stable and needs no key. Builds leaving
# out the bill of materials, which WITH_SBOM turns off, do not enable the
# experimental feature at all.
if (NOT DEFINED WITH_SBOM OR WITH_SBOM)
  # Version 4.3 reports the experimental feature by a warning meant for the
  # project, which builds failing on such warnings would fail by and
  # therefore leave out the bill of materials.
  if (CMAKE_VERSION VERSION_GREATER_EQUAL 4.3
      AND CMAKE_VERSION VERSION_LESS 4.4)
    if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_ERRORS
        OR CMAKE_SUPPRESS_DEVELOPER_ERRORS)
      set (CMAKE_EXPERIMENTAL_GENERATE_SBOM
        ca494ed3-b261-4205-a01f-603c95e4cae0)
    endif ()
  elseif (CMAKE_VERSION VERSION_GREATER_EQUAL 4.4
      AND CMAKE_VERSION VERSION_LESS 4.5)
    set (CMAKE_EXPERIMENTAL_GENERATE_SBOM
      2d856d6d-53e8-488b-a17f-d486d2cac317)
  endif ()
endif (NOT DEFINED WITH_SBOM OR WITH_SBOM)

# Builds failing on warnings meant for the project tolerate the experimental
# bill of materials. Earlier versions cannot tell experimental features
# apart.
if (COMMAND cmake_diagnostic)
  cmake_diagnostic (DEMOTE CMD_EXPERIMENTAL WARN)
endif (COMMAND cmake_diagnostic)

project (g2o
  VERSION 2017.7.30
  DESCRIPTION "General Graph Optimization"
  HOMEPAGE_URL https://github.com/RainerKuemmerle/g2o
  LANGUAGES C CXX
)

# The bundled find modules take precedence over modules of the same name
# that the user or an embedding project provides.
list (PREPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

include (CMakeDependentOption)

# CTest defines BUILD_TESTING, which only the top-level project offers.
if (PROJECT_IS_TOP_LEVEL)
  include (CTest)
endif (PROJECT_IS_TOP_LEVEL)

option (BUILD_APPS "Build g2o apps" ON)
option (BUILD_DEPRECATED_TYPES "Compile the deprecated types" OFF)
option (BUILD_EXAMPLES "Build g2o examples" ON)
option (BUILD_LGPL_SHARED_LIBS
  "Build LGPL code as shared libraries (LGPL code)" OFF)
option (BUILD_SHARED_LIBS
  "Build Shared Libraries (preferred and required for the g2o plugin system)"
  ON)
option (WITH_OPENGL "Build g2o with OpenGL support for visualization" OFF)
option (WITH_OPENMP "Build g2o with OpenMP support (EXPERIMENTAL)" OFF)
option (WITH_SBOM
  "Install the software bill of materials if CMake generates one" ON)
cmake_dependent_option (BUILD_LINKED_APPS
  "Build apps linked with the libraries (no plugin system)" OFF
  "BUILD_APPS" OFF)

find_package (CAMD CONFIG)
find_package (CHOLMOD CONFIG)
find_package (CXSparse CONFIG)

# SuiteSparse installs a package configuration per library from version 7
# on. Earlier distributions of it provide a single configuration instead.
if (NOT TARGET SuiteSparse::CHOLMOD)
  find_package (SuiteSparse 4.4.5 CONFIG
    COMPONENTS CHOLMOD
    OPTIONAL_COMPONENTS CAMD
  )
endif (NOT TARGET SuiteSparse::CHOLMOD)

find_package (Eigen3 3.4.1...5 REQUIRED CONFIG)
find_package (GTest 1.17 CONFIG)
find_package (ng-log 0.9.0 REQUIRED CONFIG COMPONENTS experimental)
find_package (OpenGL COMPONENTS OpenGL)
find_package (OpenMP COMPONENTS CXX)

# Only the viewers need Qt and QGLViewer. The module of QGLViewer searches
# for Qt itself, which therefore comes first.
if (BUILD_APPS OR BUILD_EXAMPLES)
  find_package (Qt6 COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets Xml)
  find_package (QGLViewer)
endif (BUILD_APPS OR BUILD_EXAMPLES)

include (CheckCXXCompilerFlag)
include (CheckCXXSymbolExists)
include (CheckIncludeFile)
include (CheckLibraryExists)
include (CMakePackageConfigHelpers)
include (CMakePushCheckState)
include (FeatureSummary)
include (GenerateExportHeader)
include (GNUInstallDirs)
include (GoogleTest)

check_cxx_symbol_exists (drand48 cstdlib G2O_HAVE_DRAND48)
check_cxx_symbol_exists (gettimeofday sys/time.h G2O_HAVE_GETTIMEOFDAY)
check_cxx_symbol_exists (vasprintf cstdio G2O_HAVE_VASPRINTF)
check_include_file (dlfcn.h G2O_HAVE_DLFCN_H)
check_include_file (sys/time.h G2O_HAVE_SYS_TIME_H)
check_include_file (unistd.h G2O_HAVE_UNISTD_H)
check_include_file (wordexp.h G2O_HAVE_WORDEXP_H)
check_library_exists (m pow "" G2O_HAVE_M)
check_library_exists (rt clock_gettime "" G2O_HAVE_RT)

cmake_push_check_state (RESET)
set (CMAKE_REQUIRED_LIBRARIES ${CMAKE_DL_LIBS})
check_cxx_symbol_exists (dladdr dlfcn.h G2O_HAVE_DLADDR)
cmake_pop_check_state ()

# Object files of the larger types exceed the section limit of the COFF
# format unless the assembler or the compiler raises it.
check_cxx_compiler_flag (-Wa,-mbig-obj HAVE_AR_BIGOBJ)
check_cxx_compiler_flag (/bigobj HAVE_BIGOBJ)

# SuiteSparse 7 exports CXSparse as SuiteSparse::CXSparse, earlier
# configurations as CXSparse::CXSparse.
if (TARGET SuiteSparse::CXSparse)
  set (_g2o_CXSPARSE_TARGET SuiteSparse::CXSparse)
  set (G2O_HAVE_CSPARSE ON)
elseif (TARGET CXSparse::CXSparse)
  set (_g2o_CXSPARSE_TARGET CXSparse::CXSparse)
  set (G2O_HAVE_CSPARSE ON)
endif (TARGET SuiteSparse::CXSparse)

if (WITH_OPENGL AND TARGET OpenGL::GL AND TARGET OpenGL::GLU)
  set (G2O_HAVE_OPENGL ON)
endif (WITH_OPENGL AND TARGET OpenGL::GL AND TARGET OpenGL::GLU)

# OpenMP is experimental. Eigen then does not parallelize itself, since
# that slowed the optimization down.
if (WITH_OPENMP AND TARGET OpenMP::OpenMP_CXX)
  set (G2O_OPENMP ON)
endif (WITH_OPENMP AND TARGET OpenMP::OpenMP_CXX)

if (TARGET QGLViewer::QGLViewer AND G2O_HAVE_OPENGL)
  set (_g2o_HAVE_VIEWER ON)
endif (TARGET QGLViewer::QGLViewer AND G2O_HAVE_OPENGL)

if (TARGET SuiteSparse::CHOLMOD)
  set (G2O_HAVE_CHOLMOD ON)
endif (TARGET SuiteSparse::CHOLMOD)

set (G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
set (G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
set (G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})

if (BUILD_LGPL_SHARED_LIBS)
  set (_g2o_LGPL_LIBRARY_TYPE SHARED)
else (BUILD_LGPL_SHARED_LIBS)
  set (_g2o_LGPL_LIBRARY_TYPE STATIC)
endif (BUILD_LGPL_SHARED_LIBS)

# The libraries install into a directory of their own, where the command
# line tools look for the plugins.
set (_g2o_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/g2o)
set (_g2o_INSTALL_CMAKEDIR ${_g2o_INSTALL_LIBDIR}/cmake)

# The installed programs and libraries find the libraries of g2o relative to
# their own location, so that the installation can be moved.
if (APPLE)
  set (_g2o_INSTALL_ORIGIN @loader_path)
else (APPLE)
  set (_g2o_INSTALL_ORIGIN $ORIGIN)
endif (APPLE)

cmake_path (RELATIVE_PATH CMAKE_INSTALL_FULL_LIBDIR
  BASE_DIRECTORY ${CMAKE_INSTALL_FULL_BINDIR}
  OUTPUT_VARIABLE _g2o_BINDIR_TO_LIBDIR
)

# The license of the files without a license notice of their own.
set (_g2o_DEFAULT_LICENSE BSD-2-Clause)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_BINARY_DIR}/lib)
set (CMAKE_CXX_VISIBILITY_PRESET hidden)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_BINARY_DIR}/lib)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_BINARY_DIR}/bin)
set (CMAKE_VISIBILITY_INLINES_HIDDEN ON)

if (HAVE_AR_BIGOBJ)
  add_compile_options (-Wa,-mbig-obj)
endif (HAVE_AR_BIGOBJ)

if (HAVE_BIGOBJ)
  add_compile_options (/bigobj)
endif (HAVE_BIGOBJ)

if (MSVC)
  add_compile_definitions (_CRT_SECURE_NO_WARNINGS NOMINMAX _USE_MATH_DEFINES)
  # 4244: conversion from double to int
  # 4267: conversion during return
  # 4522: duplicated operator=() in Eigen headers
  # 4661: no suitable definition for explicit template instantiation
  add_compile_options (/wd4244 /wd4267 /wd4522 /wd4661)

  if (BUILD_SHARED_LIBS)
    # 4251: missing DLL interface
    add_compile_options (/wd4251)
  endif (BUILD_SHARED_LIBS)
endif (MSVC)

configure_file (config.h.in ${g2o_BINARY_DIR}/include/g2o/config.h)

#[=======================================================================[.rst:
.. command:: g2o_add_library

  .. code-block:: cmake

    g2o_add_library (<name>
      [TYPE <STATIC|SHARED>]
      [OUTPUT_NAME <name>]
      [EXPORT_NAME <name>]
      [EXPORT_HEADER <path> EXPORT_MACRO <macro>]
      [LICENSE <expression>]
      [INSTALL <EXPORT|PROGRAM>]
      SOURCES <file>...
      [HEADERS <file>...]
    )

  Defines the library ``<name>``, whose file is named ``g2o_<name>`` or
  ``g2o_`` followed by ``OUTPUT_NAME``, and its alias ``g2o::<name>`` or
  ``g2o::`` followed by ``EXPORT_NAME``. The ``HEADERS`` form the header set
  of the library together with the export header, which
  ``generate_export_header`` writes to ``<path>`` below the include
  directory of the build tree and which defines ``G2O_<macro>``. Headers of
  the source tree form the file set ``HEADERS``, generated headers below the
  include directory of the build tree the file set ``generated_headers``,
  since the base directories of a file set must not contain each other as
  they do for a build tree inside the source tree. The headers install at
  their paths relative to their base directory. ``LICENSE`` gives the SPDX license expression of the
  sources, which defaults to the license of g2o. ``INSTALL EXPORT`` installs
  the library with the exported targets of the package, ``INSTALL PROGRAM``
  with the programs, and no ``INSTALL`` leaves it in the build tree.
#]=======================================================================]
function (g2o_add_library name)
  set (one_value_keywords
    EXPORT_HEADER
    EXPORT_MACRO
    EXPORT_NAME
    INSTALL
    LICENSE
    OUTPUT_NAME
    TYPE
  )
  cmake_parse_arguments (PARSE_ARGV 1 arg
    ""
    "${one_value_keywords}"
    "SOURCES;HEADERS"
  )

  if (arg_UNPARSED_ARGUMENTS)
    message (FATAL_ERROR "g2o_add_library (${name}) was given the unknown "
      "arguments ${arg_UNPARSED_ARGUMENTS}")
  endif (arg_UNPARSED_ARGUMENTS)

  if (DEFINED arg_INSTALL AND NOT arg_INSTALL MATCHES "^(EXPORT|PROGRAM)$")
    message (FATAL_ERROR "g2o_add_library (${name}) was given INSTALL "
      "${arg_INSTALL}, expected EXPORT or PROGRAM")
  endif (DEFINED arg_INSTALL AND NOT arg_INSTALL MATCHES "^(EXPORT|PROGRAM)$")

  if (NOT DEFINED arg_OUTPUT_NAME)
    set (arg_OUTPUT_NAME ${name})
  endif (NOT DEFINED arg_OUTPUT_NAME)

  if (NOT DEFINED arg_EXPORT_NAME)
    set (arg_EXPORT_NAME ${name})
  endif (NOT DEFINED arg_EXPORT_NAME)

  if (NOT DEFINED arg_LICENSE)
    set (arg_LICENSE ${_g2o_DEFAULT_LICENSE})
  endif (NOT DEFINED arg_LICENSE)

  add_library (${name} ${arg_TYPE} ${arg_SOURCES})
  add_library (g2o::${arg_EXPORT_NAME} ALIAS ${name})

  set (generated_dir ${g2o_BINARY_DIR}/include)
  set (headers)
  set (generated_headers)

  foreach (header IN LISTS arg_HEADERS)
    cmake_path (ABSOLUTE_PATH header BASE_DIRECTORY ${g2o_SOURCE_DIR})

    if (header MATCHES "^${generated_dir}/")
      list (APPEND generated_headers ${header})
    else (header MATCHES "^${generated_dir}/")
      list (APPEND headers ${header})
    endif (header MATCHES "^${generated_dir}/")
  endforeach (header IN LISTS arg_HEADERS)

  if (DEFINED arg_EXPORT_HEADER)
    set (export_header ${generated_dir}/${arg_EXPORT_HEADER})
    generate_export_header (${name}
      PREFIX_NAME G2O_
      EXPORT_MACRO_NAME ${arg_EXPORT_MACRO}
      EXPORT_FILE_NAME ${export_header}
    )
    list (APPEND generated_headers ${export_header})

    # A static library neither exports nor imports its symbols, which the
    # export header learns from the static define.
    get_target_property (type ${name} TYPE)

    if (type STREQUAL "STATIC_LIBRARY")
      string (MAKE_C_IDENTIFIER ${name} identifier)
      string (TOUPPER ${identifier} identifier)
      target_compile_definitions (${name} PUBLIC
        G2O_${identifier}_STATIC_DEFINE
      )
    endif (type STREQUAL "STATIC_LIBRARY")
  endif (DEFINED arg_EXPORT_HEADER)

  if (headers)
    target_sources (${name} PUBLIC
      FILE_SET HEADERS
      BASE_DIRS ${g2o_SOURCE_DIR}
      FILES ${headers}
    )
  endif (headers)

  if (generated_headers)
    target_sources (${name} PUBLIC
      FILE_SET generated_headers
      TYPE HEADERS
      BASE_DIRS ${generated_dir}
      FILES ${generated_headers}
    )
  endif (generated_headers)

  set_property (GLOBAL APPEND PROPERTY _g2o_LIBRARIES ${name})
  set_target_properties (${name} PROPERTIES
    EXPORT_NAME ${arg_EXPORT_NAME}
    OUTPUT_NAME g2o_${arg_OUTPUT_NAME}
    INSTALL_RPATH ${_g2o_INSTALL_ORIGIN}
    SOVERSION ${g2o_VERSION_MAJOR}
    SPDX_LICENSE "${arg_LICENSE}"
    VERSION ${g2o_VERSION}
  )

  if (arg_INSTALL STREQUAL "EXPORT")
    set_target_properties (${name} PROPERTIES DEBUG_POSTFIX _d)
    set (export g2o-targets)
  else (arg_INSTALL STREQUAL "EXPORT")
    set (export g2o-programs)
  endif (arg_INSTALL STREQUAL "EXPORT")

  if (DEFINED arg_INSTALL)
    set (file_set)

    if (headers)
      list (APPEND file_set FILE_SET HEADERS COMPONENT Development)
    endif (headers)

    if (generated_headers)
      list (APPEND file_set FILE_SET generated_headers COMPONENT Development)
    endif (generated_headers)

    install (TARGETS ${name}
      EXPORT ${export}
      RUNTIME COMPONENT Runtime
      LIBRARY DESTINATION ${_g2o_INSTALL_LIBDIR}
        COMPONENT Runtime
        NAMELINK_COMPONENT Development
      ARCHIVE DESTINATION ${_g2o_INSTALL_LIBDIR}
        COMPONENT Development
      ${file_set}
    )
  endif (DEFINED arg_INSTALL)
endfunction (g2o_add_library)

#[=======================================================================[.rst:
.. command:: g2o_add_executable

  .. code-block:: cmake

    g2o_add_executable (<name>
      [OUTPUT_NAME <name>]
      [LICENSE <expression>]
      [INSTALL]
      SOURCES <file>...
      [LINK <target>...]
    )

  Defines the program ``<name>``, named ``OUTPUT_NAME`` if given, from the
  ``SOURCES`` and links it to the ``LINK`` targets. ``LICENSE`` gives the
  SPDX license expression of the sources, which defaults to the license of
  g2o. ``INSTALL`` installs the program.
#]=======================================================================]
function (g2o_add_executable name)
  cmake_parse_arguments (PARSE_ARGV 1 arg
    "INSTALL"
    "OUTPUT_NAME;LICENSE"
    "SOURCES;LINK"
  )

  if (arg_UNPARSED_ARGUMENTS)
    message (FATAL_ERROR "g2o_add_executable (${name}) was given the unknown "
      "arguments ${arg_UNPARSED_ARGUMENTS}")
  endif (arg_UNPARSED_ARGUMENTS)

  if (NOT DEFINED arg_LICENSE)
    set (arg_LICENSE ${_g2o_DEFAULT_LICENSE})
  endif (NOT DEFINED arg_LICENSE)

  add_executable (${name} ${arg_SOURCES})
  target_link_libraries (${name} PRIVATE ${arg_LINK})
  set_target_properties (${name} PROPERTIES
    INSTALL_RPATH ${_g2o_INSTALL_ORIGIN}/${_g2o_BINDIR_TO_LIBDIR}/g2o
    SPDX_LICENSE "${arg_LICENSE}"
  )

  if (DEFINED arg_OUTPUT_NAME)
    set_target_properties (${name} PROPERTIES OUTPUT_NAME ${arg_OUTPUT_NAME})
  endif (DEFINED arg_OUTPUT_NAME)

  if (arg_INSTALL)
    install (TARGETS ${name} EXPORT g2o-programs RUNTIME COMPONENT Runtime)
  endif (arg_INSTALL)
endfunction (g2o_add_executable)

# Utilities

g2o_add_library (stuff
  INSTALL EXPORT
  EXPORT_HEADER g2o/stuff/g2o_stuff_api.h
  EXPORT_MACRO STUFF_API
  SOURCES
    g2o/stuff/command_args.cpp
    g2o/stuff/filesys_tools.cpp
    g2o/stuff/logger.cpp
    g2o/stuff/os_specific.cpp
    g2o/stuff/property.cpp
    g2o/stuff/sampler.cpp
    g2o/stuff/sparse_helper.cpp
    g2o/stuff/string_tools.cpp
    g2o/stuff/tictoc.cpp
    g2o/stuff/timeutil.cpp
  HEADERS
    ${g2o_BINARY_DIR}/include/g2o/config.h
    g2o/prefix.h
    g2o/stuff/color_macros.h
    g2o/stuff/command_args.h
    g2o/stuff/filesys_tools.h
    g2o/stuff/logger.h
    g2o/stuff/macros.h
    g2o/stuff/misc.h
    g2o/stuff/opengl_wrapper.h
    g2o/stuff/os_specific.h
    g2o/stuff/property.h
    g2o/stuff/sampler.h
    g2o/stuff/sparse_helper.h
    g2o/stuff/streamed.h
    g2o/stuff/string_tools.h
    g2o/stuff/tictoc.h
    g2o/stuff/timeutil.h
    g2o/stuff/unscented.h
    g2o/suffix.h
)

# The diagnostics of g2o, including those of its templates in the public
# headers, go through the experimental logging functions of ng-log, which
# compile their consumers as C++20.
target_link_libraries (stuff PUBLIC Eigen3::Eigen ng-log::experimental)

# The public headers format values that provide only a stream insertion
# operator by specializing std::formatter.
target_compile_features (stuff PUBLIC cxx_std_20)

if (G2O_HAVE_RT AND NOT ANDROID)
  target_link_libraries (stuff PRIVATE rt)
endif (G2O_HAVE_RT AND NOT ANDROID)

if (G2O_HAVE_OPENGL)
  g2o_add_library (opengl_helper
    INSTALL EXPORT
    LICENSE GPL-3.0-or-later
    EXPORT_HEADER g2o/stuff/g2o_opengl_api.h
    EXPORT_MACRO OPENGL_API
    SOURCES
      g2o/stuff/opengl_primitives.cpp
    HEADERS
      g2o/stuff/opengl_primitives.h
  )

  target_link_libraries (opengl_helper PUBLIC
    OpenGL::GL
    OpenGL::GLU
    stuff
  )

  g2o_add_library (freeglut_minimal
    INSTALL EXPORT
    OUTPUT_NAME ext_freeglut_minimal
    LICENSE MIT
    EXPORT_HEADER g2o/freeglut/g2o_freeglut_api.h
    EXPORT_MACRO FGAPI
    SOURCES
      EXTERNAL/freeglut/freeglut_font.cpp
      EXTERNAL/freeglut/freeglut_stroke_mono_roman.cpp
      EXTERNAL/freeglut/freeglut_stroke_roman.cpp
    HEADERS
      EXTERNAL/freeglut/freeglut_minimal.h
  )

  target_link_libraries (freeglut_minimal PUBLIC OpenGL::GL stuff)
endif (G2O_HAVE_OPENGL)

# Core

g2o_add_library (core
  INSTALL EXPORT
  EXPORT_HEADER g2o/core/g2o_core_api.h
  EXPORT_MACRO CORE_API
  SOURCES
    g2o/core/base_edge.cpp
    g2o/core/batch_stats.cpp
    g2o/core/cache.cpp
    g2o/core/estimate_propagator.cpp
    g2o/core/factory.cpp
    g2o/core/hyper_dijkstra.cpp
    g2o/core/hyper_graph.cpp
    g2o/core/hyper_graph_action.cpp
    g2o/core/jacobian_workspace.cpp
    g2o/core/marginal_covariance_cholesky.cpp
    g2o/core/matrix_structure.cpp
    g2o/core/optimizable_graph.cpp
    g2o/core/optimization_algorithm.cpp
    g2o/core/optimization_algorithm_dogleg.cpp
    g2o/core/optimization_algorithm_factory.cpp
    g2o/core/optimization_algorithm_gauss_newton.cpp
    g2o/core/optimization_algorithm_levenberg.cpp
    g2o/core/optimization_algorithm_with_hessian.cpp
    g2o/core/parameter.cpp
    g2o/core/parameter_container.cpp
    g2o/core/robust_kernel.cpp
    g2o/core/robust_kernel_factory.cpp
    g2o/core/robust_kernel_impl.cpp
    g2o/core/solver.cpp
    g2o/core/sparse_optimizer.cpp
    g2o/core/sparse_optimizer_terminate_action.cpp
  HEADERS
    g2o/core/abi.h
    g2o/core/base_binary_edge.h
    g2o/core/base_binary_edge.hpp
    g2o/core/base_edge.h
    g2o/core/base_multi_edge.h
    g2o/core/base_multi_edge.hpp
    g2o/core/base_unary_edge.h
    g2o/core/base_unary_edge.hpp
    g2o/core/base_vertex.h
    g2o/core/base_vertex.hpp
    g2o/core/batch_stats.h
    g2o/core/block_solver.h
    g2o/core/block_solver.hpp
    g2o/core/cache.h
    g2o/core/creators.h
    g2o/core/dynamic_aligned_buffer.hpp
    g2o/core/eigen_types.h
    g2o/core/estimate_propagator.h
    g2o/core/factory.h
    g2o/core/hyper_dijkstra.h
    g2o/core/hyper_graph.h
    g2o/core/hyper_graph_action.h
    g2o/core/jacobian_workspace.h
    g2o/core/linear_solver.h
    g2o/core/marginal_covariance_cholesky.h
    g2o/core/matrix_operations.h
    g2o/core/matrix_structure.h
    g2o/core/openmp_mutex.h
    g2o/core/optimizable_graph.h
    g2o/core/optimization_algorithm.h
    g2o/core/optimization_algorithm_dogleg.h
    g2o/core/optimization_algorithm_factory.h
    g2o/core/optimization_algorithm_gauss_newton.h
    g2o/core/optimization_algorithm_levenberg.h
    g2o/core/optimization_algorithm_property.h
    g2o/core/optimization_algorithm_with_hessian.h
    g2o/core/parameter.h
    g2o/core/parameter_container.h
    g2o/core/robust_kernel.h
    g2o/core/robust_kernel_factory.h
    g2o/core/robust_kernel_impl.h
    g2o/core/solver.h
    g2o/core/sparse_block_matrix.h
    g2o/core/sparse_block_matrix.hpp
    g2o/core/sparse_block_matrix_ccs.h
    g2o/core/sparse_block_matrix_diagonal.h
    g2o/core/sparse_optimizer.h
    g2o/core/sparse_optimizer_terminate_action.h
)

# The sources look up owned edges by their address and extract them.
target_compile_features (core PUBLIC cxx_std_11 PRIVATE cxx_std_17)
target_link_libraries (core PUBLIC Eigen3::Eigen stuff)

if (G2O_OPENMP)
  target_compile_definitions (core PUBLIC EIGEN_DONT_PARALLELIZE)
  target_link_libraries (core PUBLIC OpenMP::OpenMP_CXX)
endif (G2O_OPENMP)

# Solvers

g2o_add_library (solver_dense
  INSTALL EXPORT
  SOURCES
    g2o/solvers/dense/solver_dense.cpp
  HEADERS
    g2o/solvers/dense/linear_solver_dense.h
)

target_link_libraries (solver_dense PUBLIC core)

g2o_add_library (solver_eigen
  INSTALL EXPORT
  LICENSE "BSD-2-Clause AND LGPL-3.0-or-later"
  SOURCES
    g2o/solvers/eigen/solver_eigen.cpp
  HEADERS
    g2o/solvers/eigen/linear_solver_eigen.h
)

target_link_libraries (solver_eigen PUBLIC core)

g2o_add_library (solver_pcg
  INSTALL EXPORT
  SOURCES
    g2o/solvers/pcg/solver_pcg.cpp
  HEADERS
    g2o/solvers/pcg/linear_solver_pcg.h
    g2o/solvers/pcg/linear_solver_pcg.hpp
)

target_link_libraries (solver_pcg PUBLIC core)

g2o_add_library (solver_structure_only
  INSTALL EXPORT
  SOURCES
    g2o/solvers/structure_only/structure_only.cpp
  HEADERS
    g2o/solvers/structure_only/structure_only_solver.h
)

target_link_libraries (solver_structure_only PUBLIC core)

if (G2O_HAVE_CSPARSE)
  g2o_add_library (csparse_extension
    TYPE ${_g2o_LGPL_LIBRARY_TYPE}
    INSTALL EXPORT
    LICENSE LGPL-3.0-or-later
    EXPORT_HEADER g2o/solvers/csparse/g2o_csparse_extension_api.h
    EXPORT_MACRO CSPARSE_EXTENSION_API
    SOURCES
      g2o/solvers/csparse/csparse_helper.cpp
    HEADERS
      g2o/solvers/csparse/csparse_helper.h
  )

  target_link_libraries (csparse_extension PUBLIC
    ${_g2o_CXSPARSE_TARGET}
    core
  )

  g2o_add_library (solver_csparse
    INSTALL EXPORT
    LICENSE "BSD-2-Clause AND LGPL-3.0-or-later"
    EXPORT_HEADER g2o/solvers/csparse/g2o_solver_csparse_api.h
    EXPORT_MACRO SOLVER_CSPARSE_API
    SOURCES
      g2o/solvers/csparse/linear_solver_csparse.cpp
      g2o/solvers/csparse/solver_csparse.cpp
    HEADERS
      g2o/solvers/csparse/linear_solver_csparse.h
  )

  target_link_libraries (solver_csparse PUBLIC core csparse_extension)
endif (G2O_HAVE_CSPARSE)

if (G2O_HAVE_CHOLMOD)
  g2o_add_library (solver_cholmod
    INSTALL EXPORT
    EXPORT_HEADER g2o/solvers/cholmod/g2o_solver_cholmod_api.h
    EXPORT_MACRO SOLVER_CHOLMOD_API
    SOURCES
      g2o/solvers/cholmod/linear_solver_cholmod.cpp
      g2o/solvers/cholmod/solver_cholmod.cpp
    HEADERS
      g2o/solvers/cholmod/linear_solver_cholmod.h
  )

  target_link_libraries (solver_cholmod PUBLIC core SuiteSparse::CHOLMOD)
endif (G2O_HAVE_CHOLMOD)

# Types

g2o_add_library (types_slam2d
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/slam2d/g2o_types_slam2d_api.h
  EXPORT_MACRO TYPES_SLAM2D_API
  SOURCES
    g2o/types/slam2d/edge_pointxy.cpp
    g2o/types/slam2d/edge_se2.cpp
    g2o/types/slam2d/edge_se2_lotsofxy.cpp
    g2o/types/slam2d/edge_se2_offset.cpp
    g2o/types/slam2d/edge_se2_pointxy.cpp
    g2o/types/slam2d/edge_se2_pointxy_bearing.cpp
    g2o/types/slam2d/edge_se2_pointxy_calib.cpp
    g2o/types/slam2d/edge_se2_pointxy_offset.cpp
    g2o/types/slam2d/edge_se2_prior.cpp
    g2o/types/slam2d/edge_se2_twopointsxy.cpp
    g2o/types/slam2d/edge_se2_xyprior.cpp
    g2o/types/slam2d/parameter_se2_offset.cpp
    g2o/types/slam2d/types_slam2d.cpp
    g2o/types/slam2d/vertex_point_xy.cpp
    g2o/types/slam2d/vertex_se2.cpp
  HEADERS
    g2o/types/slam2d/edge_pointxy.h
    g2o/types/slam2d/edge_se2.h
    g2o/types/slam2d/edge_se2_lotsofxy.h
    g2o/types/slam2d/edge_se2_offset.h
    g2o/types/slam2d/edge_se2_pointxy.h
    g2o/types/slam2d/edge_se2_pointxy_bearing.h
    g2o/types/slam2d/edge_se2_pointxy_calib.h
    g2o/types/slam2d/edge_se2_pointxy_offset.h
    g2o/types/slam2d/edge_se2_prior.h
    g2o/types/slam2d/edge_se2_twopointsxy.h
    g2o/types/slam2d/edge_se2_xyprior.h
    g2o/types/slam2d/parameter_se2_offset.h
    g2o/types/slam2d/se2.h
    g2o/types/slam2d/types_slam2d.h
    g2o/types/slam2d/vertex_point_xy.h
    g2o/types/slam2d/vertex_se2.h
)

target_link_libraries (types_slam2d PUBLIC core)

g2o_add_library (types_slam3d
  INSTALL EXPORT
  LICENSE "BSD-2-Clause AND LGPL-3.0-or-later"
  EXPORT_HEADER g2o/types/slam3d/g2o_types_slam3d_api.h
  EXPORT_MACRO TYPES_SLAM3D_API
  SOURCES
    g2o/types/slam3d/dquat2mat.cpp
    g2o/types/slam3d/edge_pointxyz.cpp
    g2o/types/slam3d/edge_se3.cpp
    g2o/types/slam3d/edge_se3_lotsofxyz.cpp
    g2o/types/slam3d/edge_se3_offset.cpp
    g2o/types/slam3d/edge_se3_pointxyz.cpp
    g2o/types/slam3d/edge_se3_pointxyz_depth.cpp
    g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp
    g2o/types/slam3d/edge_se3_prior.cpp
    g2o/types/slam3d/isometry3d_gradients.cpp
    g2o/types/slam3d/isometry3d_mappings.cpp
    g2o/types/slam3d/parameter_camera.cpp
    g2o/types/slam3d/parameter_se3_offset.cpp
    g2o/types/slam3d/parameter_stereo_camera.cpp
    g2o/types/slam3d/types_slam3d.cpp
    g2o/types/slam3d/vertex_pointxyz.cpp
    g2o/types/slam3d/vertex_se3.cpp
  HEADERS
    g2o/types/slam3d/dquat2mat.h
    g2o/types/slam3d/edge_pointxyz.h
    g2o/types/slam3d/edge_se3.h
    g2o/types/slam3d/edge_se3_lotsofxyz.h
    g2o/types/slam3d/edge_se3_offset.h
    g2o/types/slam3d/edge_se3_pointxyz.h
    g2o/types/slam3d/edge_se3_pointxyz_depth.h
    g2o/types/slam3d/edge_se3_pointxyz_disparity.h
    g2o/types/slam3d/edge_se3_prior.h
    g2o/types/slam3d/isometry3d_gradients.h
    g2o/types/slam3d/isometry3d_mappings.h
    g2o/types/slam3d/parameter_camera.h
    g2o/types/slam3d/parameter_se3_offset.h
    g2o/types/slam3d/parameter_stereo_camera.h
    g2o/types/slam3d/se3_ops.h
    g2o/types/slam3d/se3_ops.hpp
    g2o/types/slam3d/se3quat.h
    g2o/types/slam3d/types_slam3d.h
    g2o/types/slam3d/vertex_pointxyz.h
    g2o/types/slam3d/vertex_se3.h
)

target_link_libraries (types_slam3d PUBLIC core)

if (G2O_HAVE_OPENGL)
  target_link_libraries (types_slam2d PUBLIC OpenGL::GL opengl_helper)
  target_link_libraries (types_slam3d PUBLIC OpenGL::GL opengl_helper)
endif (G2O_HAVE_OPENGL)

g2o_add_library (types_data
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/data/g2o_types_data_api.h
  EXPORT_MACRO TYPES_DATA_API
  SOURCES
    g2o/types/data/data_queue.cpp
    g2o/types/data/laser_parameters.cpp
    g2o/types/data/raw_laser.cpp
    g2o/types/data/robot_data.cpp
    g2o/types/data/robot_laser.cpp
    g2o/types/data/types_data.cpp
    g2o/types/data/vertex_ellipse.cpp
    g2o/types/data/vertex_tag.cpp
  HEADERS
    g2o/types/data/data_queue.h
    g2o/types/data/laser_parameters.h
    g2o/types/data/raw_laser.h
    g2o/types/data/robot_data.h
    g2o/types/data/robot_laser.h
    g2o/types/data/types_data.h
    g2o/types/data/vertex_ellipse.h
    g2o/types/data/vertex_tag.h
)

target_link_libraries (types_data PUBLIC core types_slam2d)

if (G2O_HAVE_OPENGL)
  target_link_libraries (types_data PUBLIC freeglut_minimal opengl_helper)
endif (G2O_HAVE_OPENGL)

g2o_add_library (types_sba
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/sba/g2o_types_sba_api.h
  EXPORT_MACRO TYPES_SBA_API
  SOURCES
    g2o/types/sba/types_sba.cpp
    g2o/types/sba/types_six_dof_expmap.cpp
  HEADERS
    g2o/types/sba/sbacam.h
    g2o/types/sba/types_sba.h
    g2o/types/sba/types_six_dof_expmap.h
)

target_link_libraries (types_sba PUBLIC core types_slam3d)

g2o_add_library (types_icp
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/icp/g2o_types_icp_api.h
  EXPORT_MACRO TYPES_ICP_API
  SOURCES
    g2o/types/icp/types_icp.cpp
  HEADERS
    g2o/types/icp/types_icp.h
)

target_link_libraries (types_icp PUBLIC types_sba types_slam3d)

g2o_add_library (types_sclam2d
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/sclam2d/g2o_types_sclam2d_api.h
  EXPORT_MACRO TYPES_SCLAM2D_API
  SOURCES
    g2o/types/sclam2d/edge_se2_odom_differential_calib.cpp
    g2o/types/sclam2d/edge_se2_sensor_calib.cpp
    g2o/types/sclam2d/odometry_measurement.cpp
    g2o/types/sclam2d/types_sclam2d.cpp
    g2o/types/sclam2d/vertex_odom_differential_params.cpp
  HEADERS
    g2o/types/sclam2d/edge_se2_odom_differential_calib.h
    g2o/types/sclam2d/edge_se2_sensor_calib.h
    g2o/types/sclam2d/odometry_measurement.h
    g2o/types/sclam2d/types_sclam2d.h
    g2o/types/sclam2d/vertex_odom_differential_params.h
)

target_link_libraries (types_sclam2d PUBLIC core types_slam2d)

g2o_add_library (types_sim3
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/sim3/g2o_types_sim3_api.h
  EXPORT_MACRO TYPES_SIM3_API
  SOURCES
    g2o/types/sim3/types_seven_dof_expmap.cpp
  HEADERS
    g2o/types/sim3/sim3.h
    g2o/types/sim3/types_seven_dof_expmap.h
)

target_link_libraries (types_sim3 PUBLIC types_sba)

g2o_add_library (types_slam2d_addons
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h
  EXPORT_MACRO TYPES_SLAM2D_ADDONS_API
  SOURCES
    g2o/types/slam2d_addons/edge_line2d.cpp
    g2o/types/slam2d_addons/edge_line2d_pointxy.cpp
    g2o/types/slam2d_addons/edge_se2_line2d.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d_line.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.cpp
    g2o/types/slam2d_addons/types_slam2d_addons.cpp
    g2o/types/slam2d_addons/vertex_line2d.cpp
    g2o/types/slam2d_addons/vertex_segment2d.cpp
  HEADERS
    g2o/types/slam2d_addons/edge_line2d.h
    g2o/types/slam2d_addons/edge_line2d_pointxy.h
    g2o/types/slam2d_addons/edge_se2_line2d.h
    g2o/types/slam2d_addons/edge_se2_segment2d.h
    g2o/types/slam2d_addons/edge_se2_segment2d_line.h
    g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.h
    g2o/types/slam2d_addons/line_2d.h
    g2o/types/slam2d_addons/types_slam2d_addons.h
    g2o/types/slam2d_addons/vertex_line2d.h
    g2o/types/slam2d_addons/vertex_segment2d.h
)

target_link_libraries (types_slam2d_addons PUBLIC core types_slam2d)

g2o_add_library (types_slam3d_addons
  INSTALL EXPORT
  EXPORT_HEADER g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h
  EXPORT_MACRO TYPES_SLAM3D_ADDONS_API
  SOURCES
    g2o/types/slam3d_addons/edge_plane.cpp
    g2o/types/slam3d_addons/edge_se3_calib.cpp
    g2o/types/slam3d_addons/edge_se3_euler.cpp
    g2o/types/slam3d_addons/edge_se3_line.cpp
    g2o/types/slam3d_addons/edge_se3_plane_calib.cpp
    g2o/types/slam3d_addons/line3d.cpp
    g2o/types/slam3d_addons/types_slam3d_addons.cpp
    g2o/types/slam3d_addons/vertex_line3d.cpp
    g2o/types/slam3d_addons/vertex_plane.cpp
    g2o/types/slam3d_addons/vertex_se3_euler.cpp
  HEADERS
    g2o/types/slam3d_addons/edge_plane.h
    g2o/types/slam3d_addons/edge_se3_calib.h
    g2o/types/slam3d_addons/edge_se3_euler.h
    g2o/types/slam3d_addons/edge_se3_line.h
    g2o/types/slam3d_addons/edge_se3_plane_calib.h
    g2o/types/slam3d_addons/line3d.h
    g2o/types/slam3d_addons/plane3d.h
    g2o/types/slam3d_addons/types_slam3d_addons.h
    g2o/types/slam3d_addons/vertex_line3d.h
    g2o/types/slam3d_addons/vertex_plane.h
    g2o/types/slam3d_addons/vertex_se3_euler.h
)

target_compile_definitions (types_slam3d_addons PRIVATE EIGEN_NO_STATIC_ASSERT)
target_link_libraries (types_slam3d_addons PUBLIC core types_slam3d)

if (G2O_HAVE_OPENGL)
  target_link_libraries (types_sclam2d PUBLIC OpenGL::GL)
  target_link_libraries (types_slam2d_addons PUBLIC OpenGL::GL)
  target_link_libraries (types_slam3d_addons PUBLIC OpenGL::GL)
endif (G2O_HAVE_OPENGL)

if (BUILD_DEPRECATED_TYPES)
  g2o_add_library (deprecated_types_slam3d
    INSTALL PROGRAM
    EXPORT_HEADER g2o/deprecated/g2o_deprecated_types_slam3d_api.h
    EXPORT_MACRO DEPRECATED_TYPES_SLAM3D_API
    SOURCES
      g2o/types/deprecated/slam3d/edge_se3_offset.cpp
      g2o/types/deprecated/slam3d/edge_se3_pointxyz.cpp
      g2o/types/deprecated/slam3d/edge_se3_pointxyz_depth.cpp
      g2o/types/deprecated/slam3d/edge_se3_pointxyz_disparity.cpp
      g2o/types/deprecated/slam3d/edge_se3_prior.cpp
      g2o/types/deprecated/slam3d/edge_se3_quat.cpp
      g2o/types/deprecated/slam3d/parameter_camera.cpp
      g2o/types/deprecated/slam3d/parameter_se3_offset.cpp
      g2o/types/deprecated/slam3d/se3quat_gradients.cpp
      g2o/types/deprecated/slam3d/types_slam3d.cpp
      g2o/types/deprecated/slam3d/vertex_pointxyz.cpp
      g2o/types/deprecated/slam3d/vertex_se3_quat.cpp
    HEADERS
      g2o/types/deprecated/slam3d/edge_se3_offset.h
      g2o/types/deprecated/slam3d/edge_se3_pointxyz.h
      g2o/types/deprecated/slam3d/edge_se3_pointxyz_depth.h
      g2o/types/deprecated/slam3d/edge_se3_pointxyz_disparity.h
      g2o/types/deprecated/slam3d/edge_se3_prior.h
      g2o/types/deprecated/slam3d/edge_se3_quat.h
      g2o/types/deprecated/slam3d/g2o_deprecated_types_slam3d_api.h
      g2o/types/deprecated/slam3d/parameter_camera.h
      g2o/types/deprecated/slam3d/parameter_se3_offset.h
      g2o/types/deprecated/slam3d/se3quat_gradients.h
      g2o/types/deprecated/slam3d/types_slam3d.h
      g2o/types/deprecated/slam3d/vertex_pointxyz.h
      g2o/types/deprecated/slam3d/vertex_se3_quat.h
  )

  target_link_libraries (deprecated_types_slam3d PUBLIC core types_slam3d)

  if (G2O_HAVE_OPENGL)
    target_link_libraries (deprecated_types_slam3d PUBLIC OpenGL::GL)
  endif (G2O_HAVE_OPENGL)
endif (BUILD_DEPRECATED_TYPES)

if (G2O_HAVE_CSPARSE)
  g2o_add_library (solver_slam2d_linear
    INSTALL EXPORT
    EXPORT_HEADER g2o/solvers/slam2d_linear/g2o_slam2d_linear_api.h
    EXPORT_MACRO SLAM2D_LINEAR_API
    SOURCES
      g2o/solvers/slam2d_linear/slam2d_linear.cpp
      g2o/solvers/slam2d_linear/solver_slam2d_linear.cpp
    HEADERS
      g2o/solvers/slam2d_linear/solver_slam2d_linear.h
  )

  target_link_libraries (solver_slam2d_linear PUBLIC
    solver_csparse
    types_slam2d
  )
endif (G2O_HAVE_CSPARSE)

# Applications

if (BUILD_APPS)
  g2o_add_library (g2o_cli_library
    INSTALL EXPORT
    OUTPUT_NAME cli
    EXPORT_NAME cli
    EXPORT_HEADER g2o/cli/g2o_cli_api.h
    EXPORT_MACRO CLI_API
    SOURCES
      g2o/apps/g2o_cli/dl_wrapper.cpp
      g2o/apps/g2o_cli/g2o_common.cpp
      g2o/apps/g2o_cli/output_helper.cpp
    HEADERS
      g2o/apps/g2o_cli/dl_wrapper.h
      g2o/apps/g2o_cli/g2o_common.h
      g2o/apps/g2o_cli/output_helper.h
  )

  # The plugins are searched for in the directory of the libraries of the
  # build tree, with the postfix of the configuration.
  target_compile_definitions (g2o_cli_library PRIVATE
    G2O_DEFAULT_SOLVERS_DIR_="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    G2O_DEFAULT_TYPES_DIR_="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    G2O_LIBRARY_POSTFIX=$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>$<$<CONFIG:MinSizeRel>:${CMAKE_MINSIZEREL_POSTFIX}>$<$<CONFIG:RelWithDebInfo>:${CMAKE_RELWITHDEBINFO_POSTFIX}>
  )
  target_link_libraries (g2o_cli_library
    PUBLIC core
    PRIVATE ${CMAKE_DL_LIBS}
  )

  g2o_add_executable (g2o_cli_application
    INSTALL
    OUTPUT_NAME g2o
    SOURCES g2o/apps/g2o_cli/g2o.cpp
    LINK g2o_cli_library
  )

  g2o_add_library (g2o_hierarchical_library
    INSTALL EXPORT
    OUTPUT_NAME hierarchical
    EXPORT_NAME hierarchical
    EXPORT_HEADER g2o/hierarchical/g2o_hierarchical_api.h
    EXPORT_MACRO HIERARCHICAL_API
    SOURCES
      g2o/apps/g2o_hierarchical/backbone_tree_action.cpp
      g2o/apps/g2o_hierarchical/edge_creator.cpp
      g2o/apps/g2o_hierarchical/edge_labeler.cpp
      g2o/apps/g2o_hierarchical/edge_types_cost_function.cpp
      g2o/apps/g2o_hierarchical/g2o_hierarchical_test_functions.cpp
      g2o/apps/g2o_hierarchical/simple_star_ops.cpp
      g2o/apps/g2o_hierarchical/star.cpp
    HEADERS
      g2o/apps/g2o_hierarchical/backbone_tree_action.h
      g2o/apps/g2o_hierarchical/edge_creator.h
      g2o/apps/g2o_hierarchical/edge_labeler.h
      g2o/apps/g2o_hierarchical/edge_types_cost_function.h
      g2o/apps/g2o_hierarchical/simple_star_ops.h
      g2o/apps/g2o_hierarchical/star.h
  )

  target_link_libraries (g2o_hierarchical_library PUBLIC
    core
    g2o_cli_library
  )

  g2o_add_executable (g2o_hierarchical_application
    INSTALL
    OUTPUT_NAME g2o_hierarchical
    LICENSE LGPL-3.0-or-later
    SOURCES g2o/apps/g2o_hierarchical/g2o_hierarchical.cpp
    LINK g2o_cli_library g2o_hierarchical_library types_slam3d
  )

  if (G2O_HAVE_OPENGL)
    g2o_add_library (g2o_simulator_library
      INSTALL PROGRAM
      OUTPUT_NAME simulator
      EXPORT_HEADER g2o/simulator/g2o_simulator_api.h
      EXPORT_MACRO SIMULATOR_API
      SOURCES
        g2o/apps/g2o_simulator/pointsensorparameters.cpp
        g2o/apps/g2o_simulator/sensor_odometry2d.cpp
        g2o/apps/g2o_simulator/sensor_odometry3d.cpp
        g2o/apps/g2o_simulator/sensor_pointxy.cpp
        g2o/apps/g2o_simulator/sensor_pointxy_bearing.cpp
        g2o/apps/g2o_simulator/sensor_pointxy_offset.cpp
        g2o/apps/g2o_simulator/sensor_pointxyz.cpp
        g2o/apps/g2o_simulator/sensor_pointxyz_depth.cpp
        g2o/apps/g2o_simulator/sensor_pointxyz_disparity.cpp
        g2o/apps/g2o_simulator/sensor_pose2d.cpp
        g2o/apps/g2o_simulator/sensor_pose3d.cpp
        g2o/apps/g2o_simulator/sensor_pose3d_offset.cpp
        g2o/apps/g2o_simulator/sensor_se3_prior.cpp
        g2o/apps/g2o_simulator/sensor_segment2d.cpp
        g2o/apps/g2o_simulator/sensor_segment2d_line.cpp
        g2o/apps/g2o_simulator/sensor_segment2d_pointline.cpp
        g2o/apps/g2o_simulator/simulator.cpp
        g2o/apps/g2o_simulator/simutils.cpp
      HEADERS
        g2o/apps/g2o_simulator/g2o_simulator_api.h
        g2o/apps/g2o_simulator/pointsensorparameters.h
        g2o/apps/g2o_simulator/sensor_line3d.h
        g2o/apps/g2o_simulator/sensor_odometry.h
        g2o/apps/g2o_simulator/sensor_odometry2d.h
        g2o/apps/g2o_simulator/sensor_odometry3d.h
        g2o/apps/g2o_simulator/sensor_pointxy.h
        g2o/apps/g2o_simulator/sensor_pointxy_bearing.h
        g2o/apps/g2o_simulator/sensor_pointxy_offset.h
        g2o/apps/g2o_simulator/sensor_pointxyz.h
        g2o/apps/g2o_simulator/sensor_pointxyz_depth.h
        g2o/apps/g2o_simulator/sensor_pointxyz_disparity.h
        g2o/apps/g2o_simulator/sensor_pose2d.h
        g2o/apps/g2o_simulator/sensor_pose3d.h
        g2o/apps/g2o_simulator/sensor_pose3d_offset.h
        g2o/apps/g2o_simulator/sensor_se3_prior.h
        g2o/apps/g2o_simulator/sensor_segment2d.h
        g2o/apps/g2o_simulator/sensor_segment2d_line.h
        g2o/apps/g2o_simulator/sensor_segment2d_pointline.h
        g2o/apps/g2o_simulator/simulator.h
        g2o/apps/g2o_simulator/simulator2d.h
        g2o/apps/g2o_simulator/simulator2d_base.h
        g2o/apps/g2o_simulator/simulator3d.h
        g2o/apps/g2o_simulator/simulator3d_base.h
        g2o/apps/g2o_simulator/simutils.h
    )

    target_link_libraries (g2o_simulator_library PUBLIC
      core
      types_slam2d
      types_slam2d_addons
      types_slam3d
      types_slam3d_addons
    )

    g2o_add_executable (g2o_simulator2d_application
      INSTALL
      OUTPUT_NAME g2o_simulator2d
      SOURCES g2o/apps/g2o_simulator/test_simulator2d.cpp
      LINK g2o_simulator_library
    )

    g2o_add_executable (g2o_simulator3d_application
      INSTALL
      OUTPUT_NAME g2o_simulator3d
      SOURCES g2o/apps/g2o_simulator/test_simulator3d.cpp
      LINK g2o_simulator_library
    )

    g2o_add_executable (g2o_anonymize_observations_application
      INSTALL
      OUTPUT_NAME g2o_anonymize_observations
      SOURCES g2o/apps/g2o_simulator/g2o_anonymize_observations.cpp
      LINK core types_slam2d types_slam3d
    )

    g2o_add_executable (convertSegmentLine_application
      OUTPUT_NAME convertSegmentLine
      SOURCES g2o/apps/g2o_simulator/convertSegmentLine.cpp
      LINK g2o_simulator_library
    )
  endif (G2O_HAVE_OPENGL)

  if (_g2o_HAVE_VIEWER)
    g2o_add_library (viewer_library
      INSTALL PROGRAM
      OUTPUT_NAME viewer
      LICENSE "BSD-2-Clause AND GPL-3.0-or-later AND LGPL-3.0-or-later"
      EXPORT_HEADER g2o/viewer/g2o_viewer_api.h
      EXPORT_MACRO VIEWER_API
      SOURCES
        g2o/apps/g2o_viewer/base_main_window.ui
        g2o/apps/g2o_viewer/base_properties_widget.ui
        g2o/apps/g2o_viewer/g2o_qglviewer.cpp
        g2o/apps/g2o_viewer/gui_hyper_graph_action.cpp
        g2o/apps/g2o_viewer/main_window.cpp
        g2o/apps/g2o_viewer/properties_widget.cpp
        g2o/apps/g2o_viewer/run_g2o_viewer.cpp
        g2o/apps/g2o_viewer/stream_redirect.cpp
        g2o/apps/g2o_viewer/text_edit_sink.cpp
        g2o/apps/g2o_viewer/viewer_properties_widget.cpp
      HEADERS
        g2o/apps/g2o_viewer/g2o_qglviewer.h
        g2o/apps/g2o_viewer/gui_hyper_graph_action.h
        g2o/apps/g2o_viewer/main_window.h
        g2o/apps/g2o_viewer/properties_widget.h
        g2o/apps/g2o_viewer/run_g2o_viewer.h
        g2o/apps/g2o_viewer/stream_redirect.h
        g2o/apps/g2o_viewer/text_edit_sink.h
        g2o/apps/g2o_viewer/viewer_properties_widget.h
    )

    # The headers generated from the forms include the viewer headers by
    # their file names.
    set_target_properties (viewer_library PROPERTIES
      AUTOMOC ON
      AUTOUIC ON
    )
    target_include_directories (viewer_library PUBLIC
      $<BUILD_INTERFACE:${g2o_SOURCE_DIR}/g2o/apps/g2o_viewer>
    )
    target_link_libraries (viewer_library PUBLIC
      core
      g2o_cli_library
      OpenGL::GL
      OpenGL::GLU
      opengl_helper
      QGLViewer::QGLViewer
      Qt6::Core
      Qt6::Gui
      Qt6::OpenGL
      Qt6::OpenGLWidgets
      Qt6::Widgets
      Qt6::Xml
    )

    g2o_add_executable (g2o_viewer
      INSTALL
      LICENSE GPL-3.0-or-later
      SOURCES g2o/apps/g2o_viewer/g2o_viewer.cpp
      LINK viewer_library
    )
  endif (_g2o_HAVE_VIEWER)

  # The linked programs load no libraries at run time and are linked to the
  # solvers and types instead.
  if (BUILD_LINKED_APPS)
    set (_g2o_LINKED_LIBRARIES
      solver_pcg
      types_sba
      types_sclam2d
      types_slam2d
      types_slam3d
    )

    foreach (_g2o_SOLVER IN ITEMS solver_cholmod solver_csparse)
      if (TARGET ${_g2o_SOLVER})
        list (APPEND _g2o_LINKED_LIBRARIES ${_g2o_SOLVER})
      endif (TARGET ${_g2o_SOLVER})
    endforeach (_g2o_SOLVER IN ITEMS solver_cholmod solver_csparse)

    g2o_add_executable (g2o_cli_application_linked
      INSTALL
      OUTPUT_NAME g2o_linked
      SOURCES g2o/apps/g2o_cli/g2o.cpp
      LINK g2o_cli_library ${_g2o_LINKED_LIBRARIES}
    )

    target_compile_definitions (g2o_cli_application_linked PRIVATE
      G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
    )

    if (_g2o_HAVE_VIEWER)
      g2o_add_executable (g2o_viewer_linked
        INSTALL
        LICENSE GPL-3.0-or-later
        SOURCES g2o/apps/g2o_viewer/g2o_viewer.cpp
        LINK viewer_library ${_g2o_LINKED_LIBRARIES}
      )

      target_compile_definitions (g2o_viewer_linked PRIVATE
        G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
      )
    endif (_g2o_HAVE_VIEWER)
  endif (BUILD_LINKED_APPS)
endif (BUILD_APPS)

# Examples

if (BUILD_EXAMPLES)
  g2o_add_executable (curve_fit
    SOURCES g2o/examples/data_fitting/curve_fit.cpp
    LINK core
  )

  g2o_add_executable (create_sphere
    SOURCES g2o/examples/sphere/create_sphere.cpp
    LINK core types_slam3d
  )

  g2o_add_executable (convert_sba_slam3d
    SOURCES g2o/examples/data_convert/convert_sba_slam3d.cpp
    LINK core types_sba types_slam3d
  )

  # The parser of the interactive SLAM includes its headers relative to the
  # directory of the interactive SLAM examples.
  g2o_add_library (parser_library
    INSTALL EXPORT
    OUTPUT_NAME parser
    EXPORT_NAME parser
    LICENSE "BSD-2-Clause AND BSD-3-Clause AND GPL-3.0-or-later WITH Bison-exception-2.2"
    EXPORT_HEADER g2o/slam_parser/g2o_slam_parser_api.h
    EXPORT_MACRO SLAM_PARSER_API
    SOURCES
      g2o/examples/interactive_slam/slam_parser/parser/bison_parser.cpp
      g2o/examples/interactive_slam/slam_parser/parser/driver.cpp
      g2o/examples/interactive_slam/slam_parser/parser/flex_scanner.cpp
      g2o/examples/interactive_slam/slam_parser/parser/slam_context.cpp
    HEADERS
      g2o/examples/interactive_slam/slam_parser/parser/FlexLexer.h
      g2o/examples/interactive_slam/slam_parser/parser/bison_parser.h
      g2o/examples/interactive_slam/slam_parser/parser/commands.h
      g2o/examples/interactive_slam/slam_parser/parser/driver.h
      g2o/examples/interactive_slam/slam_parser/parser/location.hh
      g2o/examples/interactive_slam/slam_parser/parser/scanner.h
      g2o/examples/interactive_slam/slam_parser/parser/slam_context.h
  )

  # The generated scanner includes the bundled FlexLexer.h with angle
  # brackets, which must take precedence over the header of an installed
  # Flex, whose interface differs.
  target_link_libraries (parser_library PRIVATE stuff)
  target_include_directories (parser_library
    PUBLIC
      $<BUILD_INTERFACE:${g2o_SOURCE_DIR}/g2o/examples/interactive_slam>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/g2o/examples/interactive_slam>
    PRIVATE
      ${g2o_SOURCE_DIR}/g2o/examples/interactive_slam/slam_parser/parser
  )

  g2o_add_library (interface_library
    INSTALL EXPORT
    OUTPUT_NAME interface
    EXPORT_NAME interface
    EXPORT_HEADER g2o/slam_parser/interface/g2o_slam_parser_interface_api.h
    EXPORT_MACRO SLAM_PARSER_INTERFACE_API
    SOURCES
      g2o/examples/interactive_slam/slam_parser/interface/parser_interface.cpp
      g2o/examples/interactive_slam/slam_parser/interface/slam_context_interface.cpp
    HEADERS
      g2o/examples/interactive_slam/slam_parser/interface/abstract_slam_interface.h
      g2o/examples/interactive_slam/slam_parser/interface/parser_interface.h
      g2o/examples/interactive_slam/slam_parser/interface/slam_context_interface.h
  )

  target_link_libraries (interface_library
    PUBLIC parser_library
    PRIVATE stuff
  )

  g2o_add_library (example_library
    OUTPUT_NAME example
    EXPORT_HEADER g2o/slam_parser/example/g2o_example_api.h
    EXPORT_MACRO EXAMPLE_API
    SOURCES
      g2o/examples/interactive_slam/slam_parser/example/example_slam_interface.cpp
    HEADERS
      g2o/examples/interactive_slam/slam_parser/example/example_slam_interface.h
  )

  target_link_libraries (example_library
    PUBLIC
      interface_library
      parser_library
    PRIVATE stuff
  )

  g2o_add_executable (test_slam_parser
    SOURCES g2o/examples/interactive_slam/slam_parser/parser/test_slam_parser.cpp
    LINK parser_library stuff
  )

  g2o_add_executable (test_slam_interface
    SOURCES g2o/examples/interactive_slam/slam_parser/example/test_slam_interface.cpp
    LINK example_library stuff
  )

  if (G2O_HAVE_CSPARSE)
    g2o_add_executable (circle_fit
      SOURCES g2o/examples/data_fitting/circle_fit.cpp
      LINK core solver_csparse
    )

    g2o_add_library (tutorial_slam2d_library
      OUTPUT_NAME tutorial_slam2d
      EXPORT_HEADER g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h
      EXPORT_MACRO TUTORIAL_SLAM2D_API
      SOURCES
        g2o/examples/tutorial_slam2d/edge_se2.cpp
        g2o/examples/tutorial_slam2d/edge_se2_pointxy.cpp
        g2o/examples/tutorial_slam2d/parameter_se2_offset.cpp
        g2o/examples/tutorial_slam2d/simulator.cpp
        g2o/examples/tutorial_slam2d/types_tutorial_slam2d.cpp
        g2o/examples/tutorial_slam2d/vertex_point_xy.cpp
        g2o/examples/tutorial_slam2d/vertex_se2.cpp
      HEADERS
        g2o/examples/tutorial_slam2d/edge_se2.h
        g2o/examples/tutorial_slam2d/edge_se2_pointxy.h
        g2o/examples/tutorial_slam2d/parameter_se2_offset.h
        g2o/examples/tutorial_slam2d/rand.h
        g2o/examples/tutorial_slam2d/se2.h
        g2o/examples/tutorial_slam2d/simulator.h
        g2o/examples/tutorial_slam2d/types_tutorial_slam2d.h
        g2o/examples/tutorial_slam2d/vertex_point_xy.h
        g2o/examples/tutorial_slam2d/vertex_se2.h
    )

    target_link_libraries (tutorial_slam2d_library PUBLIC core solver_csparse)

    g2o_add_executable (tutorial_slam2d
      SOURCES g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
      LINK tutorial_slam2d_library
    )

    g2o_add_executable (gicp_demo
      SOURCES g2o/examples/icp/gicp_demo.cpp
      LINK core solver_csparse types_icp types_sba types_slam3d
    )

    g2o_add_executable (gicp_sba_demo
      SOURCES g2o/examples/icp/gicp_sba_demo.cpp
      LINK core solver_csparse types_icp types_sba types_slam3d
    )

    g2o_add_library (calibration_odom_laser_library
      OUTPUT_NAME calibration_odom_laser
      EXPORT_HEADER g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h
      EXPORT_MACRO CALIBRATION_ODOM_LASER_API
      SOURCES
        g2o/examples/calibration_odom_laser/closed_form_calibration.cpp
        g2o/examples/calibration_odom_laser/edge_se2_pure_calib.cpp
        g2o/examples/calibration_odom_laser/gm2dl_io.cpp
        g2o/examples/calibration_odom_laser/sclam_helpers.cpp
      HEADERS
        g2o/examples/calibration_odom_laser/closed_form_calibration.h
        g2o/examples/calibration_odom_laser/edge_se2_pure_calib.h
        g2o/examples/calibration_odom_laser/gm2dl_io.h
        g2o/examples/calibration_odom_laser/motion_information.h
        g2o/examples/calibration_odom_laser/sclam_helpers.h
    )

    target_link_libraries (calibration_odom_laser_library PUBLIC
      core
      solver_csparse
      types_data
      types_sclam2d
      types_slam2d
    )

    g2o_add_executable (sclam_laser_calib
      SOURCES g2o/examples/calibration_odom_laser/sclam_laser_calib.cpp
      LINK calibration_odom_laser_library
    )

    g2o_add_executable (sclam_odom_laser
      SOURCES g2o/examples/calibration_odom_laser/sclam_odom_laser.cpp
      LINK calibration_odom_laser_library
    )

    g2o_add_executable (sclam_pure_calibration
      SOURCES g2o/examples/calibration_odom_laser/sclam_pure_calibration.cpp
      LINK calibration_odom_laser_library
    )

    g2o_add_executable (simple_optimize
      SOURCES g2o/examples/simple_optimize/simple_optimize.cpp
      LINK core solver_csparse types_slam2d types_slam3d
    )

    g2o_add_executable (simulator_3d_plane
      SOURCES g2o/examples/plane_slam/simulator_3d_plane.cpp
      LINK solver_csparse types_slam3d_addons
    )

    g2o_add_executable (simulator_3d_line
      SOURCES g2o/examples/line_slam/simulator_3d_line.cpp
      LINK solver_csparse types_slam3d_addons
    )
  endif (G2O_HAVE_CSPARSE)

  # These examples use the CHOLMOD solver explicitly.
  if (G2O_HAVE_CHOLMOD)
    g2o_add_executable (static_target
      SOURCES g2o/examples/target/static_target.cpp
      LINK core solver_cholmod
    )

    g2o_add_executable (constant_velocity_target
      SOURCES g2o/examples/target/constant_velocity_target.cpp
      LINK core solver_cholmod solver_pcg types_slam2d
    )

    g2o_add_executable (ba_demo
      SOURCES g2o/examples/ba/ba_demo.cpp
      LINK core solver_cholmod types_sba
    )

    g2o_add_executable (ba_anchored_inverse_depth_demo
      SOURCES g2o/examples/ba_anchored_inverse_depth/ba_anchored_inverse_depth_demo.cpp
      LINK core solver_cholmod types_sba
    )

    g2o_add_library (g2o_interactive_library
      INSTALL PROGRAM
      OUTPUT_NAME interactive
      EXPORT_HEADER g2o/examples/interactive_slam/g2o_interactive_api.h
      EXPORT_MACRO INTERACTIVE_API
      SOURCES
        g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.cpp
        g2o/examples/interactive_slam/g2o_interactive/graph_optimizer_sparse_online.cpp
        g2o/examples/interactive_slam/g2o_interactive/types_online.cpp
      HEADERS
        g2o/examples/interactive_slam/g2o_interactive/fast_output.h
        g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h
        g2o/examples/interactive_slam/g2o_interactive/graph_optimizer_sparse_online.h
        g2o/examples/interactive_slam/g2o_interactive/types_slam2d_online.h
        g2o/examples/interactive_slam/g2o_interactive/types_slam3d_online.h
    )

    target_link_libraries (g2o_interactive_library PUBLIC
      core
      interface_library
      parser_library
      solver_cholmod
      types_slam2d
      types_slam3d
    )

    g2o_add_executable (g2o_online_application
      INSTALL
      OUTPUT_NAME g2o_online
      SOURCES g2o/examples/interactive_slam/g2o_interactive/g2o_online.cpp
      LINK g2o_interactive_library
    )

    if (TARGET SuiteSparse::CAMD)
      g2o_add_library (g2o_incremental_library
        INSTALL PROGRAM
        OUTPUT_NAME incremental
        LICENSE GPL-3.0-or-later
        EXPORT_HEADER g2o/examples/interactive_slam/g2o_incremental_api.h
        EXPORT_MACRO INCREMENTAL_API
        SOURCES
          g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.cpp
        HEADERS
          g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h
          g2o/examples/interactive_slam/g2o_incremental/linear_solver_cholmod_online.h
      )

      target_link_libraries (g2o_incremental_library PUBLIC
        g2o_interactive_library
        solver_cholmod
        SuiteSparse::CAMD
      )

      g2o_add_executable (g2o_incremental_application
        INSTALL
        OUTPUT_NAME g2o_incremental
        LICENSE GPL-3.0-or-later
        SOURCES g2o/examples/interactive_slam/g2o_incremental/g2o_incremental.cpp
        LINK g2o_incremental_library
      )
    endif (TARGET SuiteSparse::CAMD)

    g2o_add_executable (sba_demo
      SOURCES g2o/examples/sba/sba_demo.cpp
      LINK core solver_cholmod types_icp types_sba
    )

    g2o_add_executable (bal_example
      SOURCES g2o/examples/bal/bal_example.cpp
      LINK solver_cholmod
    )
  endif (G2O_HAVE_CHOLMOD)

  if (_g2o_HAVE_VIEWER AND G2O_HAVE_CSPARSE)
    g2o_add_executable (slam2d_g2o
      LICENSE GPL-3.0-or-later
      SOURCES
        g2o/examples/slam2d/base_main_window.ui
        g2o/examples/slam2d/draw_helpers.cpp
        g2o/examples/slam2d/draw_helpers.h
        g2o/examples/slam2d/main_window.cpp
        g2o/examples/slam2d/main_window.h
        g2o/examples/slam2d/slam2d_g2o.cpp
        g2o/examples/slam2d/slam2d_viewer.cpp
        g2o/examples/slam2d/slam2d_viewer.h
      LINK
        core
        OpenGL::GL
        OpenGL::GLU
        QGLViewer::QGLViewer
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::Xml
        solver_csparse
        types_slam2d
    )

    # The header generated from the form includes the viewer header by its
    # file name.
    set_target_properties (slam2d_g2o PROPERTIES
      AUTOMOC ON
      AUTOUIC ON
    )
    target_include_directories (slam2d_g2o PRIVATE
      ${g2o_SOURCE_DIR}/g2o/examples/slam2d
    )
  endif (_g2o_HAVE_VIEWER AND G2O_HAVE_CSPARSE)
endif (BUILD_EXAMPLES)

# Tests

if (BUILD_TESTING AND TARGET GTest::gtest_main)
  g2o_add_executable (unittest_general
    SOURCES
      unit_test/general/clear_and_redo.cpp
      unit_test/general/factory.cpp
      unit_test/general/graph_operations.cpp
      unit_test/general/hyper_graph_action.cpp
      unit_test/general/logging.cpp
      unit_test/general/ownership.cpp
    LINK GTest::gmock GTest::gtest_main types_data types_sba types_sim3
      types_slam2d types_slam3d
  )

  g2o_add_executable (unittest_slam2d
    SOURCES
      unit_test/slam2d/jacobians_slam2d.cpp
      unit_test/slam2d/mappings_se2.cpp
      unit_test/slam2d/odom_convert_sclam2d.cpp
    LINK GTest::gmock GTest::gtest_main types_sclam2d types_slam2d
  )

  g2o_add_executable (unittest_slam3d
    SOURCES
      unit_test/slam3d/jacobians_slam3d.cpp
      unit_test/slam3d/mappings_slam3d.cpp
      unit_test/slam3d/orthogonal_matrix.cpp
    LINK GTest::gmock GTest::gtest_main types_slam3d
  )

  gtest_discover_tests (unittest_general)
  gtest_discover_tests (unittest_slam2d)
  gtest_discover_tests (unittest_slam3d)
endif (BUILD_TESTING AND TARGET GTest::gtest_main)

# The installed package is tested the way consumers use it: a prefix inside
# the build tree receives the installation, and a project of its own finds
# the package there and links to every library it exports.
if (BUILD_TESTING)
  set (_g2o_CONSUMER_PREFIX ${g2o_BINARY_DIR}/consumer-prefix)
  set (_g2o_CONSUMER_BINARY_DIR ${g2o_BINARY_DIR}/consumer)
  set (_g2o_CONSUMER_PREFIX_PATH ${_g2o_CONSUMER_PREFIX} ${CMAKE_PREFIX_PATH})
  list (JOIN _g2o_CONSUMER_PREFIX_PATH "$<SEMICOLON>" _g2o_CONSUMER_PREFIX_PATH)

  set (_g2o_CONSUMER_OPTIONS
    -DCMAKE_BUILD_TYPE=$<CONFIG>
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
    -DCMAKE_PREFIX_PATH=${_g2o_CONSUMER_PREFIX_PATH}
  )

  if (CMAKE_TOOLCHAIN_FILE)
    list (APPEND _g2o_CONSUMER_OPTIONS
      -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    )
  endif (CMAKE_TOOLCHAIN_FILE)

  # The consumer requests every library of the package that this build creates
  # as a component and includes each header of a component against that
  # component alone, so that a library missing from the package or a missing
  # usage requirement fails the test.
  set (_g2o_PACKAGE_COMPONENTS
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
  get_property (_g2o_LIBRARIES GLOBAL PROPERTY _g2o_LIBRARIES)
  set (_g2o_CONSUMER_COMPONENTS)
  set (_g2o_CONSUMER_MANIFEST)

  foreach (_g2o_TARGET IN LISTS _g2o_LIBRARIES)
    get_target_property (_g2o_COMPONENT ${_g2o_TARGET} EXPORT_NAME)

    if (NOT _g2o_COMPONENT IN_LIST _g2o_PACKAGE_COMPONENTS)
      continue ()
    endif (NOT _g2o_COMPONENT IN_LIST _g2o_PACKAGE_COMPONENTS)

    get_target_property (_g2o_HEADERS ${_g2o_TARGET} HEADER_SET)
    set (_g2o_INCLUDES)

    foreach (_g2o_HEADER IN LISTS _g2o_HEADERS)
      # The scanner includes the header of Flex after naming its class, so the
      # header is not meant to be included on its own.
      if (_g2o_HEADER MATCHES "/FlexLexer\\.h$")
        continue ()
      endif (_g2o_HEADER MATCHES "/FlexLexer\\.h$")

      # A header includes its template definitions inside its namespace.
      string (REGEX REPLACE "\\.hpp$" ".h" _g2o_DECLARATIONS "${_g2o_HEADER}")

      if (NOT _g2o_DECLARATIONS STREQUAL _g2o_HEADER
          AND _g2o_DECLARATIONS IN_LIST _g2o_HEADERS)
        continue ()
      endif (NOT _g2o_DECLARATIONS STREQUAL _g2o_HEADER
        AND _g2o_DECLARATIONS IN_LIST _g2o_HEADERS)

      cmake_path (ABSOLUTE_PATH _g2o_HEADER BASE_DIRECTORY ${g2o_SOURCE_DIR})
      cmake_path (RELATIVE_PATH _g2o_HEADER BASE_DIRECTORY ${g2o_SOURCE_DIR}
        OUTPUT_VARIABLE _g2o_INCLUDE
      )
      list (APPEND _g2o_INCLUDES ${_g2o_INCLUDE})
    endforeach (_g2o_HEADER IN LISTS _g2o_HEADERS)

    list (APPEND _g2o_CONSUMER_COMPONENTS ${_g2o_COMPONENT})
    string (APPEND _g2o_CONSUMER_MANIFEST
      "set (_consumer_HEADERS_${_g2o_COMPONENT} \"${_g2o_INCLUDES}\")\n"
    )
  endforeach (_g2o_TARGET IN LISTS _g2o_LIBRARIES)

  string (APPEND _g2o_CONSUMER_MANIFEST
    "set (_consumer_COMPONENTS \"${_g2o_CONSUMER_COMPONENTS}\")\n"
  )
  file (CONFIGURE OUTPUT ${g2o_BINARY_DIR}/consumer-manifest.cmake
    CONTENT "${_g2o_CONSUMER_MANIFEST}"
  )
  list (APPEND _g2o_CONSUMER_OPTIONS
    -D_consumer_MANIFEST=${g2o_BINARY_DIR}/consumer-manifest.cmake
  )

  add_test (NAME g2o_install_clean
    COMMAND ${CMAKE_COMMAND} -E rm -rf
      ${_g2o_CONSUMER_PREFIX}
      ${_g2o_CONSUMER_BINARY_DIR}
  )
  add_test (NAME g2o_install
    COMMAND ${CMAKE_COMMAND} --install ${g2o_BINARY_DIR}
      --prefix ${_g2o_CONSUMER_PREFIX}
      --config $<CONFIG>
  )
  add_test (NAME g2o_consumer
    COMMAND ${CMAKE_CTEST_COMMAND}
      --build-and-test ${g2o_SOURCE_DIR}/unit_test/consumer
        ${_g2o_CONSUMER_BINARY_DIR}
      --build-generator ${CMAKE_GENERATOR}
      --build-config $<CONFIG>
      --build-options ${_g2o_CONSUMER_OPTIONS}
      --test-command ${CMAKE_CTEST_COMMAND} --output-on-failure
        --build-config $<CONFIG>
  )

  set_tests_properties (g2o_install_clean PROPERTIES
    FIXTURES_SETUP g2o_consumer_clean
  )
  set_tests_properties (g2o_install PROPERTIES
    FIXTURES_REQUIRED g2o_consumer_clean
    FIXTURES_SETUP g2o_installed
  )
  set_tests_properties (g2o_consumer PROPERTIES
    FIXTURES_REQUIRED g2o_installed
  )

  # The package rejects a component that no library provides. Only the message
  # of the package passes, since configuring fails for other reasons as well.
  add_test (NAME g2o_unknown_component
    COMMAND ${CMAKE_COMMAND}
      -S ${g2o_SOURCE_DIR}/unit_test/consumer/unknown_component
      -B ${g2o_BINARY_DIR}/unknown-component
      -G ${CMAKE_GENERATOR}
      ${_g2o_CONSUMER_OPTIONS}
  )
  set_tests_properties (g2o_unknown_component PROPERTIES
    FIXTURES_REQUIRED g2o_installed
    PASS_REGULAR_EXPRESSION
      "component[ \n]+no_such_component[ \n]+could[ \n]+not[ \n]+be[ \n]+found"
  )

  # The installed programs find the libraries of g2o and load its plugins on
  # their own. Dependencies installed elsewhere, such as ng-log, are found
  # through the search path of the environment.
  # The plugin of the data types depends on the one of the 2D types, which
  # the program loads after it.
  if (BUILD_APPS AND UNIX AND NOT APPLE)
    # Each plugin lists an entry that only it provides. A plugin that fails to
    # load fails every test, and the plugins of the build tree, which
    # G2O_TYPES_DIR and G2O_SOLVERS_DIR may name, stay out of the tests.
    set (_g2o_INSTALLED_PLUGINS
      solver_cholmod listSolvers "gn_var_cholmod"
      solver_csparse listSolvers "gn_var[ \t]"
      solver_dense listSolvers "gn_dense"
      solver_eigen listSolvers "gn_var_eigen"
      solver_pcg listSolvers "gn_pcg"
      solver_slam2d_linear listSolvers "2dlinear"
      solver_structure_only listSolvers "structure_only_2"
      types_data listTypes "VERTEX_TAG"
      types_icp listTypes "EDGE_V_V_GICP"
      types_sba listTypes "VERTEX_SE3:EXPMAP"
      types_sclam2d listTypes "VERTEX_ODOM_DIFFERENTIAL"
      types_sim3 listTypes "VERTEX_SIM3:EXPMAP"
      types_slam2d listTypes "VERTEX_SE2[\r\n]"
      types_slam2d_addons listTypes "VERTEX_SEGMENT2D"
      types_slam3d listTypes "VERTEX_SE3:QUAT"
      types_slam3d_addons listTypes "VERTEX_PLANE"
    )

    while (_g2o_INSTALLED_PLUGINS)
      list (POP_FRONT _g2o_INSTALLED_PLUGINS _g2o_PLUGIN _g2o_OPTION
        _g2o_MARKER
      )

      if (NOT TARGET ${_g2o_PLUGIN})
        continue ()
      endif (NOT TARGET ${_g2o_PLUGIN})

      add_test (NAME g2o_installed_${_g2o_PLUGIN}
        COMMAND ${_g2o_CONSUMER_PREFIX}/${CMAKE_INSTALL_BINDIR}/g2o
          -${_g2o_OPTION}
      )
      set_tests_properties (g2o_installed_${_g2o_PLUGIN} PROPERTIES
        ENVIRONMENT_MODIFICATION
          "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:ng-log::ng-log>;G2O_TYPES_DIR=unset:;G2O_SOLVERS_DIR=unset:"
        FAIL_REGULAR_EXPRESSION "Cannot open library"
        FIXTURES_REQUIRED g2o_installed
        PASS_REGULAR_EXPRESSION "${_g2o_MARKER}"
      )
    endwhile (_g2o_INSTALLED_PLUGINS)
  endif (BUILD_APPS AND UNIX AND NOT APPLE)

  # In incremental mode, the command line tool takes the vertices and edges out
  # of the graph it loaded and adds them back one by one. The tool exits
  # successfully even when it cannot allocate the solver, add an element or
  # factorize the system, so these messages fail the test.
  if (BUILD_APPS AND BUILD_EXAMPLES)
    # A sphere of 5 laps with 20 poses each has 99 odometry edges and 220 loop
    # closures. The incremental run reports the size of the graph and its chi2
    # after each step with -v. The last report shows the complete graph, whose
    # chi2 stays below 10000 for the fixed seed of the sphere.
    add_test (NAME g2o_sphere_clean
      COMMAND ${CMAKE_COMMAND} -E rm -f ${g2o_BINARY_DIR}/sphere.g2o
    )
    add_test (NAME g2o_sphere
      COMMAND create_sphere -laps 5 -nodesPerLevel 20
        -o ${g2o_BINARY_DIR}/sphere.g2o
    )
    add_test (NAME g2o_incremental
      COMMAND g2o_cli_application -v -inc -update 10
        ${g2o_BINARY_DIR}/sphere.g2o
    )
    set_tests_properties (g2o_sphere_clean PROPERTIES
      FIXTURES_SETUP g2o_sphere_clean
    )
    set_tests_properties (g2o_sphere PROPERTIES
      FIXTURES_REQUIRED g2o_sphere_clean
      FIXTURES_SETUP g2o_sphere
    )
    set_tests_properties (g2o_incremental PROPERTIES
      FAIL_REGULAR_EXPRESSION
        "Error allocating;Error adding vertex;Unable to add edge;initialization failed;Cholesky failure"
      FIXTURES_REQUIRED g2o_sphere
      PASS_REGULAR_EXPRESSION
        "nodes= 100\t edges= 319\t chi2= [0-9][0-9]?[0-9]?[0-9]?\\."
    )
  endif (BUILD_APPS AND BUILD_EXAMPLES)
endif (BUILD_TESTING)

# A directory that is not part of the repository may hold experiments.
if (IS_DIRECTORY ${g2o_SOURCE_DIR}/g2o/playground)
  option (BUILD_PLAYGROUND "Build g2o playground" ON)

  if (BUILD_PLAYGROUND)
    message (STATUS "building playground")
    add_subdirectory (g2o/playground)
  endif (BUILD_PLAYGROUND)
endif (IS_DIRECTORY ${g2o_SOURCE_DIR}/g2o/playground)

# Installation and export

# The package configuration finds the dependencies of the exported
# libraries, including the private ones of static libraries.
set (_g2o_FIND_DEPENDENCIES
  "find_dependency (Eigen3 ${Eigen3_VERSION} CONFIG)"
  "find_dependency (ng-log ${ng-log_VERSION} CONFIG COMPONENTS experimental)"
)

if (G2O_HAVE_CSPARSE)
  list (APPEND _g2o_FIND_DEPENDENCIES
    "find_dependency (CXSparse ${CXSparse_VERSION} CONFIG)"
  )
endif (G2O_HAVE_CSPARSE)

# The version tells whether CHOLMOD came with a configuration of its own.
if (DEFINED CHOLMOD_VERSION)
  list (APPEND _g2o_FIND_DEPENDENCIES
    "find_dependency (CHOLMOD ${CHOLMOD_VERSION} CONFIG)"
  )
elseif (G2O_HAVE_CHOLMOD)
  list (APPEND _g2o_FIND_DEPENDENCIES
    "find_dependency (SuiteSparse ${SuiteSparse_VERSION} CONFIG COMPONENTS CHOLMOD)"
  )
endif (DEFINED CHOLMOD_VERSION)

if (G2O_HAVE_OPENGL)
  list (APPEND _g2o_FIND_DEPENDENCIES
    "find_dependency (OpenGL COMPONENTS OpenGL)"
  )
endif (G2O_HAVE_OPENGL)

if (G2O_OPENMP)
  list (APPEND _g2o_FIND_DEPENDENCIES
    "find_dependency (OpenMP COMPONENTS CXX)"
  )
endif (G2O_OPENMP)

list (JOIN _g2o_FIND_DEPENDENCIES "\n  " g2o_FIND_DEPENDENCIES)

configure_package_config_file (g2o-config.cmake.in
  ${g2o_BINARY_DIR}/g2o-config.cmake
  INSTALL_DESTINATION ${_g2o_INSTALL_CMAKEDIR}
  NO_SET_AND_CHECK_MACRO
)
write_basic_package_version_file (${g2o_BINARY_DIR}/g2o-config-version.cmake
  COMPATIBILITY SameMajorVersion
)

install (FILES
  ${g2o_BINARY_DIR}/g2o-config.cmake
  ${g2o_BINARY_DIR}/g2o-config-version.cmake
  DESTINATION ${_g2o_INSTALL_CMAKEDIR}
  COMPONENT Development
)
install (EXPORT g2o-targets
  NAMESPACE g2o::
  DESTINATION ${_g2o_INSTALL_CMAKEDIR}
  COMPONENT Development
)

if (_g2o_HAVE_VIEWER)
  # The installed viewer headers include the headers generated from the
  # forms, which the automatic user interface compiler writes per
  # configuration for generators with several configurations.
  get_property (_g2o_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

  if (_g2o_IS_MULTI_CONFIG)
    set (_g2o_VIEWER_UI_DIR
      ${g2o_BINARY_DIR}/viewer_library_autogen/include_$<CONFIG>)
  else (_g2o_IS_MULTI_CONFIG)
    set (_g2o_VIEWER_UI_DIR ${g2o_BINARY_DIR}/viewer_library_autogen/include)
  endif (_g2o_IS_MULTI_CONFIG)

  install (FILES
    ${_g2o_VIEWER_UI_DIR}/ui_base_main_window.h
    ${_g2o_VIEWER_UI_DIR}/ui_base_properties_widget.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/apps/g2o_viewer
    COMPONENT Development
  )
endif (_g2o_HAVE_VIEWER)

export (EXPORT g2o-targets
  NAMESPACE g2o::
  FILE ${g2o_BINARY_DIR}/g2o-targets.cmake
)

# Versions of CMake that generate no bill of materials leave the key of the
# experimental feature unset.
if (WITH_SBOM AND (CMAKE_VERSION VERSION_GREATER_EQUAL 4.5
    OR CMAKE_EXPERIMENTAL_GENERATE_SBOM))
  # The bill of materials describes the exported libraries and the installed
  # programs, whose licenses their targets state. Its own data is licensed
  # under CC0-1.0, as SPDX recommends, which versions before 4.5 take as
  # LICENSE and write as the license of the data as well.
  if (CMAKE_VERSION VERSION_GREATER_EQUAL 4.5)
    set (_g2o_SBOM_ARGUMENTS
      EXPORTS g2o-targets g2o-programs
      DATA_LICENSE CC0-1.0
      DEFAULT_LICENSE ${_g2o_DEFAULT_LICENSE}
      PACKAGE_URL ${g2o_HOMEPAGE_URL}/releases
    )
  elseif (CMAKE_VERSION VERSION_GREATER_EQUAL 4.4)
    set (_g2o_SBOM_ARGUMENTS
      EXPORTS g2o-targets g2o-programs
      LICENSE CC0-1.0
      PACKAGE_URL ${g2o_HOMEPAGE_URL}/releases
    )
  else (CMAKE_VERSION VERSION_GREATER_EQUAL 4.5)
    # Version 4.3 describes a single export set, which is the one of the
    # libraries.
    set (_g2o_SBOM_ARGUMENTS
      EXPORT g2o-targets
      LICENSE CC0-1.0
    )
  endif (CMAKE_VERSION VERSION_GREATER_EQUAL 4.5)

  install (SBOM g2o
    ${_g2o_SBOM_ARGUMENTS}
    DESTINATION ${CMAKE_INSTALL_DATADIR}/sbom
  )
endif ()

set_package_properties (CXSparse PROPERTIES
  URL "https://people.engr.tamu.edu/davis/suitesparse.html"
  DESCRIPTION "A concise sparse Cholesky factorization package"
  TYPE RECOMMENDED
  PURPOSE "Sparse matrix decompositions"
)
set_package_properties (Eigen3 PROPERTIES
  URL "https://eigen.tuxfamily.org"
  DESCRIPTION "C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms"
  TYPE REQUIRED
  PURPOSE "Provides fundamental linear algebra functionality"
)
set_package_properties (ng-log PROPERTIES
  URL "https://github.com/ng-log/ng-log"
  DESCRIPTION "C++ logging library"
  TYPE REQUIRED
  PURPOSE "Logs the diagnostics of g2o"
)
set_package_properties (QGLViewer PROPERTIES
  URL "https://libqglviewer.com"
  DESCRIPTION "C++ library based on Qt that eases the creation of OpenGL 3D viewers"
  TYPE OPTIONAL
  PURPOSE "Required for visualization tools"
)
set_package_properties (Qt6 PROPERTIES
  URL "https://www.qt.io/developers"
  DESCRIPTION "A cross-platform application and UI framework"
  TYPE OPTIONAL
  PURPOSE "Required for visualization tools"
)
set_package_properties (CHOLMOD PROPERTIES
  URL "https://people.engr.tamu.edu/davis/suitesparse.html"
  DESCRIPTION "Sparse Cholesky factorization of SuiteSparse"
  TYPE RECOMMENDED
  PURPOSE "Sparse matrix decompositions"
)
set_package_properties (SuiteSparse PROPERTIES
  URL "https://people.engr.tamu.edu/davis/suitesparse.html"
  DESCRIPTION "A sparse suite of matrix software"
  TYPE RECOMMENDED
  PURPOSE "Sparse matrix decompositions"
)

add_feature_info (Examples BUILD_EXAMPLES "Demonstration of g2o features")
add_feature_info (OpenGL G2O_HAVE_OPENGL "Visualization")
add_feature_info (OpenMP G2O_OPENMP "Multithreading support")
add_feature_info (SBOM WITH_SBOM "Software bill of materials")
add_feature_info (Tests BUILD_TESTING "Unit tests")
add_feature_info (Tools BUILD_APPS "A set of additional tools")

if (PROJECT_IS_TOP_LEVEL)
  feature_summary (WHAT ALL)

  include (CPack)
endif (PROJECT_IS_TOP_LEVEL)
