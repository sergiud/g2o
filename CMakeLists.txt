cmake_minimum_required (VERSION 3.1)
set (CMAKE_LEGACY_CYGWIN_WIN32 0)

project (g2o)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list (APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

include(CMakeDependentOption)

find_package (CSparse)

if (CSPARSE_FOUND)
  option (BUILD_CSPARSE "Build local CSparse library" OFF)
else (CSPARSE_FOUND)
  option (BUILD_CSPARSE "Build local CSparse library" ON)

  if (BUILD_CSPARSE)
    set (CSPARSE_FOUND TRUE)
  endif (BUILD_CSPARSE)
endif (CSPARSE_FOUND)

option (BUILD_APPS "Build g2o apps" ON)
option (BUILD_EXAMPLES "Build g2o examples" ON)
option (BUILD_LGPL_SHARED_LIBS "Build LGPL Code as Shared Libraries (LGPL Code)" ON)
option (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
option (BUILD_WITH_MARCH_NATIVE "Build with \"-march native\"" ON)
option (USE_OPENGL "Build g2o with OpenGL support for visualization" OFF)
option (USE_OPENMP "Build g2o with OpenMP support (EXPERIMENTAL)" OFF)
cmake_dependent_option (BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "BUILD_APPS" OFF)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

# The library prefix
set(LIB_PREFIX g2o_)

set(g2o_C_FLAGS)
set(g2o_CXX_FLAGS)

# postfix, based on type
set (CMAKE_DEBUG_POSTFIX "_d")
set (CMAKE_RELEASE_POSTFIX "")
set (CMAKE_RELWITHDEBINFO_POSTFIX "_rd")
set (CMAKE_MINSIZEREL_POSTFIX "_s")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
SET (G2O_LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  SET (G2O_LIB_TYPE SHARED)
ENDIF()

if (MSVC)
  add_definitions (-D_CRT_SECURE_NO_WARNINGS)
endif (MSVC)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(g2o_RUNTIME_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
IF(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})


# Detect OS and define macros appropriately
IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  IF(MSVC)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    # SSE2 optimizations
    ADD_DEFINITIONS("/arch:SSE2")
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
    ENDIF()
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
FIND_PACKAGE (SuiteSparse 4.4.5 CONFIG REQUIRED)

SET (G2O_LGPL_LIB_TYPE STATIC)
IF (BUILD_LGPL_SHARED_LIBS)
  SET (G2O_LGPL_LIB_TYPE SHARED)
ENDIF()

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)

IF(OPENMP_FOUND AND USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
ENDIF(OPENMP_FOUND AND USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND AND USE_OPENGL)
  SET (G2O_HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
ENDIF()

# For building the GUI
FIND_PACKAGE(QGLViewer)

# shall we build the core apps using the library

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  IF(NOT ${ARCH} MATCHES arm)
    # Generic settings for optimisation
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4.2")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4.2")
  ENDIF()
  # switch off optimization for debug builds
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(BUILD_WITH_MARCH_NATIVE AND NOT ${ARCH} MATCHES arm AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  ENDIF()
  # activate warnings !!!
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # activate all warnings
  #SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Weverything")
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Weverything")
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall")
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -stdlib=libc++")
ENDIF()

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})

# Generate config.h
SET(G2O_OPENGL_FOUND ${OPENGL_FOUND})
SET(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL)
ADD_SUBDIRECTORY(g2o)
