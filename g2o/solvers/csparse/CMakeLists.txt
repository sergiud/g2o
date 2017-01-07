ADD_LIBRARY(csparse_extension ${G2O_LGPL_LIB_TYPE}
  csparse_helper.cpp csparse_helper.h
)

generate_export_header (csparse_extension PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/csparse/g2o_csparse_extension_api.h
  EXPORT_MACRO_NAME CSPARSE_EXTENSION_API)

target_include_directories (csparse_extension PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${g2o_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(csparse_extension PROPERTIES OUTPUT_NAME ${LIB_PREFIX}csparse_extension)

target_link_libraries (csparse_extension PUBLIC core)

IF(BUILD_CSPARSE)
  TARGET_LINK_LIBRARIES(csparse_extension PUBLIC csparse)
ELSE()
  TARGET_LINK_LIBRARIES(csparse_extension PUBLIC CXSparse::CXSparse)
ENDIF()

ADD_LIBRARY(solver_csparse
  solver_csparse.cpp
  linear_solver_csparse.h
)

generate_export_header (solver_csparse PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/csparse/g2o_solver_csparse_api.h
  EXPORT_MACRO_NAME SOLVER_CSPARSE_API)

target_include_directories (solver_csparse PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(solver_csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_csparse)

TARGET_LINK_LIBRARIES(solver_csparse PUBLIC csparse_extension core)

INSTALL(TARGETS solver_csparse csparse_extension EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers}
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/csparse/g2o_csparse_extension_api.h
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/csparse/g2o_solver_csparse_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/csparse)
