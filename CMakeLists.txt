cmake_minimum_required (VERSION 3.1)

if (POLICY CMP0022)
  cmake_policy (SET CMP0022 NEW)
endif (POLICY CMP0022)

if (POLICY CMP0058)
  cmake_policy (SET CMP0058 NEW)
endif (POLICY CMP0058)

if (POLICY CMP0063)
  cmake_policy (SET CMP0063 NEW)
endif (POLICY CMP0063)

project (g2o)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list (APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

set (MAJOR_VERSION 2017)
set (MINOR_VERSION 7)
set (VERSION_PATCH 30)

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

include (CheckCXXCompilerFlag)
include (CheckCXXSymbolExists)
include (CheckFunctionExists)
include (CheckIncludeFile)
include (CheckLibraryExists)
include (CMakeDependentOption)
include (CMakePackageConfigHelpers)
include (CMakePushCheckState)
include (CPack)
include (CTest)
include (FeatureSummary)
include (GenerateExportHeader)
include (GNUInstallDirs)

check_cxx_symbol_exists (drand48 cstdlib G2O_HAVE_DRAND48)
check_cxx_symbol_exists (vasprintf cstdio G2O_HAVE_VASPRINTF)
check_include_file (dlfcn.h G2O_HAVE_DLFCN_H)
check_include_file (sys/time.h G2O_HAVE_SYS_TIME_H)
check_include_file (unistd.h G2O_HAVE_UNISTD_H)
check_include_file (wordexp.h G2O_HAVE_WORDEXP_H)
check_library_exists (m pow math.h G2O_HAVE_M)
check_library_exists (rt clock_gettime time.h G2O_HAVE_RT)
check_library_exists (dl dlopen dlfcn.h G2O_HAVE_DL)

cmake_push_check_state ()
set (CMAKE_REQUIRED_INCLUDE dlfcn.h)
set (CMAKE_REQUIRED_LIBRARIES dl)
check_function_exists (dladdr G2O_HAVE_DLADDR)
cmake_pop_check_state ()

cmake_push_check_state ()
set (CMAKE_REQUIRED_INCLUDE sys/time.h)
set (CMAKE_REQUIRED_LIBRARIES)
check_function_exists (gettimeofday G2O_HAVE_GETTIMEOFDAY)
cmake_pop_check_state ()

# GCC compiler flags
check_cxx_compiler_flag (-Wa,-mbig-obj HAVE_AR_BIGOBJ)

# MSVC compiler flags
check_cxx_compiler_flag (/bigobj HAVE_BIGOBJ)

if (HAVE_AR_BIGOBJ)
  add_compile_options (-Wa,-mbig-obj)
endif (HAVE_AR_BIGOBJ)

if (HAVE_BIGOBJ)
  add_compile_options (/bigobj)
endif (HAVE_BIGOBJ)

set (_g2o_CMake_Base_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
set (_g2o_CMake_BINDIR ${CMAKE_INSTALL_BINDIR})
set (_g2o_CMake_INCLUDEDIR ${_g2o_CMake_Base_INCLUDEDIR}/g2o)
set (_g2o_CMake_LIBDIR ${CMAKE_INSTALL_LIBDIR}/g2o)
set (_g2o_CMake_INSTALLDIR ${_g2o_CMake_LIBDIR}/cmake)

if (DEFINED BUILD_CSPARSE AND NOT BUILD_CSPARSE)
  # BUILD_CSPARSE is set to OFF, require CXSparse
  find_package (CXSparse 3.1 REQUIRED NO_MODULE)
elseif (DEFINED BUILD_CSPARSE AND BUILD_CSPARSE)
  # BUILD_CSPARSE is set to ON, we don't need CXSparse
else (DEFINED BUILD_CSPARSE AND NOT BUILD_CSPARSE)
  # In other cases, we just try to find CXSParse and decide from there if we
  # build CSparse or use CXSparse.
  find_package (CXSparse 3.1 NO_MODULE)
endif (DEFINED BUILD_CSPARSE AND NOT BUILD_CSPARSE)

if (NOT TARGET CXSparse::CXSparse)
  option (BUILD_CSPARSE "Build local CSparse library" ON)
else (NOT TARGET CXSparse::CXSparse)
  option (BUILD_CSPARSE "Build local CSparse library" OFF)
endif (NOT TARGET CXSparse::CXSparse)

if (BUILD_CSPARSE OR TARGET CXSparse::CXSparse)
  set (CSPARSE_FOUND TRUE)
else (BUILD_CSPARSE OR TARGET CXSparse::CXSparse)
  set (CSPARSE_FOUND FALSE)
endif (BUILD_CSPARSE OR TARGET CXSparse::CXSparse)

option (BUILD_APPS "Build g2o apps" ON)
option (BUILD_EXAMPLES "Build g2o examples" ON)
option (BUILD_LGPL_SHARED_LIBS "Build LGPL code as shared libraries (LGPL code)" OFF)
option (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
option (WITH_OPENGL "Build g2o with OpenGL support for visualization" OFF)
option (WITH_OPENMP "Build g2o with OpenMP support (EXPERIMENTAL)" OFF)
cmake_dependent_option (BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "BUILD_APPS" OFF)
option (BUILD_TESTING "build unit test framework and the tests" ON)

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
    add_definitions("/wd4661") # no suitable definition provided for explicit template instantiation
  ENDIF(MSVC)
ENDIF(WIN32)

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

if (BUILD_APPS)
  if (WITH_OPENGL)
    # OpenGL is used in the draw actions for the different types, as well
    # as for creating the GUI itself
    FIND_PACKAGE(OpenGL)

    IF (OPENGL_FOUND)
      SET (G2O_HAVE_OPENGL 1)
    ENDIF (OPENGL_FOUND)
  endif (WITH_OPENGL)

  find_package (Qt5 COMPONENTS Core Gui OpenGL Widgets Xml)

  # For building the GUI
  FIND_PACKAGE(QGLViewer)
endif (BUILD_APPS)

# Find Eigen3
FIND_PACKAGE (Eigen3 REQUIRED)

# Generate config.h
SET(G2O_HAVE_CHOLMOD ${SuiteSparse_FOUND})
SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

configure_file(config.h.in ${CMAKE_BINARY_DIR}/include/g2o/config.h)

install(FILES
  ${PROJECT_BINARY_DIR}/include/g2o/config.h
  ${g2o_SOURCE_DIR}/g2o/prefix.h
  ${g2o_SOURCE_DIR}/g2o/suffix.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR})

# Put binaries in the top-level build directory
set (OUTPUT_DIR ${CMAKE_BINARY_DIR})

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)

# building unit test framework and our tests
IF(BUILD_TESTING)
  ADD_SUBDIRECTORY(unit_test)
ENDIF()

# Include the subdirectories
if (BUILD_APPS)
  add_subdirectory(EXTERNAL)
endif (BUILD_APPS)

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

if (TARGET g2o_hierarchical_library)
  list (APPEND g2o_TARGETS g2o_hierarchical_library)
endif (TARGET g2o_hierarchical_library)

if (TARGET g2o_cli_library)
  list (APPEND g2o_TARGETS g2o_cli_library)
endif (TARGET g2o_cli_library)

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

if (TARGET freeglut_minimal)
  list (APPEND g2o_TARGETS freeglut_minimal)
endif (TARGET freeglut_minimal)

if (TARGET opengl_helper)
  list (APPEND g2o_TARGETS opengl_helper)
endif (TARGET opengl_helper)

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

set_package_properties (CXSparse PROPERTIES
  URL "http://faculty.cse.tamu.edu/davis/suitesparse.html"
  DESCRIPTION "A concise sparse Cholesky factorization package"
  TYPE RECOMMENDED
  PURPOSE "Sparse matrix decompositions"
)

set_package_properties (SuiteSparse PROPERTIES
  URL "http://faculty.cse.tamu.edu/davis/suitesparse.html"
  DESCRIPTION "A sparse suite of matrix software"
  TYPE RECOMMENDED
  PURPOSE "Sparse matrix decompositions"
)

set_package_properties (QGLViewer PROPERTIES
  URL "http://libqglviewer.com"
  DESCRIPTION "C++ library based on Qt that eases the creation of OpenGL 3D viewers"
  TYPE OPTIONAL
  PURPOSE "Required for visualization tools"
)

set_package_properties (Qt5 PROPERTIES
  URL "https://www.qt.io/developers"
  DESCRIPTION "C++ template library for task parallelism"
  TYPE OPTIONAL
  PURPOSE "Required for visualization tools"
)

set_package_properties (Qt5Core PROPERTIES
  DESCRIPTION "A cross-platform application and UI framework"
  TYPE REQUIRED
)

set_package_properties (Qt5Xml PROPERTIES
  DESCRIPTION "XML support"
  TYPE REQUIRED
)

set_package_properties (Qt5OpenGL PROPERTIES
  DESCRIPTION "OpenGL support"
  TYPE REQUIRED
)

set_package_properties (Qt5Gui PROPERTIES
  DESCRIPTION "Graphical user interface functionality"
  TYPE REQUIRED
)

set_package_properties (Qt5Widgets PROPERTIES
  DESCRIPTION "Graphical user interface widgets"
  TYPE REQUIRED
)

set_package_properties (Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  DESCRIPTION "C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms"
  TYPE REQUIRED
  PURPOSE "Provides fundamental linear algebra functionality"
)

add_feature_info (Examples BUILD_EXAMPLES "Demonstration of g2o features")
add_feature_info (OpenGL WITH_OPENGL "Visualization")
add_feature_info (OpenMP WITH_OPENMP "Multithreading support")
add_feature_info (Tools BUILD_APPS "A set of additional tools")
add_feature_info (Tests BUILD_TESTING "Unit tests")

feature_summary (WHAT ALL)
