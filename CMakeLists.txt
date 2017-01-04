cmake_minimum_required (VERSION 3.1)

if (POLICY CMP0063)
  cmake_policy (SET CMP0063 NEW)
endif (POLICY CMP0063)

project (g2o)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list (APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

set (MAJOR_VERSION 0)
set (MINOR_VERSION 1)
set (VERSION_PATCH 0)

set (SHORT_VERSION "${MAJOR_VERSION}.${MINOR_VERSION}")

if (NOT VERSION_PATCH EQUAL 0)
  set (SHORT_VERSION "${SHORT_VERSION}.${VERSION_PATCH}")
endif (NOT VERSION_PATCH EQUAL 0)

set (VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${VERSION_PATCH}")

set (CPACK_PACKAGE_NAME g2o)
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "General Graph Optimization")
set (CPACK_PACKAGE_VERSION_MAJOR ${MAJOR_VERSION})
set (CPACK_PACKAGE_VERSION_MINOR ${MINOR_VERSION})
set (CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set (CPACK_PACKAGE_VERSION ${SHORT_VERSION})

include (CMakeDependentOption)
include (CMakePackageConfigHelpers)
include (CPack)
include (GenerateExportHeader)
include (GNUInstallDirs)

set (_g2o_CMake_Base_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
set (_g2o_CMake_BINDIR ${CMAKE_INSTALL_BINDIR})
set (_g2o_CMake_INCLUDEDIR ${_g2o_CMake_Base_INCLUDEDIR}/g2o)
set (_g2o_CMake_LIBDIR ${CMAKE_INSTALL_LIBDIR}/g2o)
set (_g2o_CMake_INSTALLDIR ${_g2o_CMake_LIBDIR}/cmake)

find_package (CXSparse 3.1)

if (TARGET CXSparse::CXSparse)
  option (BUILD_CSPARSE "Build local CSparse library" OFF)
else (TARGET CXSparse::CXSparse)
  option (BUILD_CSPARSE "Build local CSparse library" ON)

  if (BUILD_CSPARSE)
    set (CSPARSE_FOUND TRUE)
  endif (BUILD_CSPARSE)
endif (TARGET CXSparse::CXSparse)

option (BUILD_APPS "Build g2o apps" ON)
option (BUILD_EXAMPLES "Build g2o examples" ON)
option (BUILD_LGPL_SHARED_LIBS "Build LGPL code as shared libraries (LGPL code)" OFF)
option (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
option (WITH_OPENGL "Build g2o with OpenGL support for visualization" OFF)
option (WITH_OPENMP "Build g2o with OpenMP support (EXPERIMENTAL)" OFF)
cmake_dependent_option (BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "BUILD_APPS" OFF)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set (CMAKE_CXX_VISIBILITY_PRESET hidden)
set (CMAKE_VISIBILITY_INLINES_HIDDEN 1)

set (CMAKE_POSITION_INDEPENDENT_CODE ON)

# The library prefix
set(LIB_PREFIX g2o_)

if (MSVC)
  add_definitions (-D_CRT_SECURE_NO_WARNINGS)
endif (MSVC)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)


# Detect OS and define macros appropriately
IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  IF(MSVC)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
    ENDIF()
    add_definitions("/wd4244") # Conversion from double -> int
    add_definitions("/wd4267") # Conversion during return
    add_definitions("/wd4522") # Duplicated operator=() in Eigen headers
  ENDIF(MSVC)
ELSEIF(CYGWIN)
  ADD_DEFINITIONS(-DCYGWIN)
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
ENDIF(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
IF(ANDROID)
  ADD_DEFINITIONS(-DANDROID)
ENDIF()

# For building the CHOLMOD / CSPARSE solvers
find_package (SuiteSparse 4.4.5 NO_MODULE)

SET (G2O_LGPL_LIB_TYPE STATIC)
IF (BUILD_LGPL_SHARED_LIBS)
  SET (G2O_LGPL_LIB_TYPE SHARED)
ENDIF()

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)

IF(OPENMP_FOUND AND WITH_OPENMP)
  SET (G2O_OPENMP 1)
  add_compile_options (-DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS})
ENDIF(OPENMP_FOUND AND WITH_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND AND WITH_OPENGL)
  SET (G2O_HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
ENDIF()

# For building the GUI
FIND_PACKAGE(QGLViewer)

# Find Eigen3
FIND_PACKAGE (Eigen3 REQUIRED)

# Generate config.h
SET(G2O_OPENGL_FOUND ${OPENGL_FOUND})
SET(G2O_HAVE_CHOLMOD ${SuiteSparse_FOUND})
SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)

install(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${_g2o_CMake_INCLUDEDIR})

# Put binaries in the top-level build directory
set (OUTPUT_DIR ${CMAKE_BINARY_DIR})

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)

# Include the subdirectories
add_subdirectory(EXTERNAL)
add_subdirectory(g2o)

if (SuiteSparse_FOUND)
  set (SuiteSparse_DEPENDENCY
    "find_dependency (SuiteSparse ${SuiteSparse_VERSION})")
endif (SuiteSparse_FOUND)

if (TARGET CXSparse::CXSparse)
  set (CXSparse_DEPENDENCY
    "find_dependency (CXSparse ${CXSparse_VERSION})")
endif (TARGET CXSparse::CXSparse)

if (TARGET Eigen3::Eigen)
  set (Eigen3_DEPENDENCY
    "find_dependency (Eigen3 ${Eigen3_VERSION})")
endif (TARGET Eigen3::Eigen)

# g2o export targets
set (g2o_TARGETS
  core
  g2o_cli_library
  g2o_hierarchical_library
  solver_dense
  solver_pcg
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

if (TARGET csparse)
  list (APPEND g2o_TARGETS csparse)
endif (TARGET csparse)

if (TARGET csparse_extension)
  list (APPEND g2o_TARGETS csparse_extension)
endif (TARGET csparse_extension)

if (TARGET solver_slam2d_linear)
  list (APPEND g2o_TARGETS solver_slam2d_linear)
endif (TARGET solver_slam2d_linear)

if (TARGET solver_csparse)
  list (APPEND g2o_TARGETS solver_csparse)
endif (TARGET solver_csparse)

if (TARGET solver_cholmod)
  list (APPEND g2o_TARGETS solver_cholmod)
endif (TARGET solver_cholmod)

if (TARGET solver_eigen)
  list (APPEND g2o_TARGETS solver_eigen)
endif (TARGET solver_eigen)

set_target_properties (${g2o_TARGETS} PROPERTIES DEBUG_POSTFIX _d)

configure_package_config_file (g2o-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/g2o-config.cmake
  INSTALL_DESTINATION ${_g2o_CMake_INSTALLDIR})
write_basic_package_version_file (g2o-config-version.cmake
  VERSION ${VERSION} COMPATIBILITY SameMajorVersion)

set (g2o_NAMESPACE g2o::)

export (TARGETS ${g2o_TARGETS} NAMESPACE ${g2o_NAMESPACE} FILE
  g2o-targets.cmake)
export (PACKAGE g2o)

install (EXPORT g2o-targets NAMESPACE ${g2o_NAMESPACE}
  DESTINATION ${_g2o_CMake_INSTALLDIR})
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/g2o-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/g2o-config-version.cmake
  DESTINATION ${_g2o_CMake_INSTALLDIR})
