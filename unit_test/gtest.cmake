########################### GTEST
# Enable ExternalProject CMake module
INCLUDE(ExternalProject)

# Set default ExternalProject root directory
SET_DIRECTORY_PROPERTIES(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/third_party)

set(GTEST_LIBRARY_PATH ${CMAKE_BINARY_DIR}/third_party/src/googletest-build/${CMAKE_STATIC_LIBRARY_PREFIX}gtest${CMAKE_STATIC_LIBRARY_SUFFIX})

# Add gtest
# http://stackoverflow.com/questions/9689183/cmake-googletest
ExternalProject_Add(
  googletest
  URL https://github.com/google/googletest/archive/release-1.7.0.zip
  URL_MD5 ef5e700c8a0f3ee123e2e0209b8b4961
  # # Force separate output paths for debug and release builds to allow easy
  # # identification of correct lib in subsequent TARGET_LINK_LIBRARIES commands
  # CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
  #            -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
  #            -Dgtest_force_shared_crt=ON
  # Disable install step
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${GTEST_LIBRARY_PATH}
  # Wrap download, configure and build steps in a script to log output
)

# Specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_DIR ${source_dir}/include)

# Library
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBRARY google::gtest)
add_library(google::gtest STATIC IMPORTED)
set_property(TARGET google::gtest PROPERTY IMPORTED_LOCATION ${GTEST_LIBRARY_PATH})
add_dependencies (google::gtest googletest)

# some dependencies for linking
if(UNIX)
  set(GTEST_LIBRARIES ${GTEST_LIBRARY} pthread)
else()
  set(GTEST_LIBRARIES ${GTEST_LIBRARY})
endif()
