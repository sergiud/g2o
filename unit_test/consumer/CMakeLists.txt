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
# The build that installed g2o lists the libraries it exports, which are the
# components to request, and the headers of each.
if (NOT DEFINED _consumer_MANIFEST)
  message (FATAL_ERROR "_consumer_MANIFEST is not defined, expected the path "
    "of the manifest that the build of g2o writes")
endif (NOT DEFINED _consumer_MANIFEST)

include (${_consumer_MANIFEST})

get_cmake_property (_consumer_VARIABLES_BEFORE VARIABLES)

foreach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_BEFORE)
  set ("_consumer_BEFORE_${_consumer_VARIABLE}" "${${_consumer_VARIABLE}}")
endforeach (_consumer_VARIABLE IN LISTS _consumer_VARIABLES_BEFORE)

find_package (g2o REQUIRED COMPONENTS ${_consumer_COMPONENTS})

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

add_executable (consumer consumer.cpp)

foreach (_consumer_COMPONENT IN LISTS _consumer_COMPONENTS)
  string (TOUPPER ${_consumer_COMPONENT} _consumer_DEFINITION)
  target_compile_definitions (consumer PRIVATE
    G2O_CONSUMER_HAVE_${_consumer_DEFINITION}
  )
  target_link_libraries (consumer PRIVATE g2o::${_consumer_COMPONENT})
  message (STATUS "Consuming g2o::${_consumer_COMPONENT}")

  # Each header compiles with the usage requirements of its library alone.
  set (_consumer_SOURCE
    ${CMAKE_CURRENT_BINARY_DIR}/headers_${_consumer_COMPONENT}.cpp
  )
  set (_consumer_CONTENT)

  foreach (_consumer_HEADER IN LISTS _consumer_HEADERS_${_consumer_COMPONENT})
    string (APPEND _consumer_CONTENT "#include <${_consumer_HEADER}>\n")
  endforeach (_consumer_HEADER IN LISTS _consumer_HEADERS_${_consumer_COMPONENT})

  file (CONFIGURE OUTPUT ${_consumer_SOURCE} CONTENT "${_consumer_CONTENT}")
  add_library (headers_${_consumer_COMPONENT} OBJECT ${_consumer_SOURCE})
  target_link_libraries (headers_${_consumer_COMPONENT} PRIVATE
    g2o::${_consumer_COMPONENT}
  )
endforeach (_consumer_COMPONENT IN LISTS _consumer_COMPONENTS)

enable_testing ()
add_test (NAME consumer COMMAND consumer)
