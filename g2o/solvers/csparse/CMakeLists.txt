ADD_LIBRARY(csparse_extension ${G2O_LGPL_LIB_TYPE}
  csparse_helper.cpp csparse_helper.h
)

generate_export_header (csparse_extension EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_csparse_extension_api.h
  EXPORT_MACRO_NAME G2O_CSPARSE_EXTENSION_API)

target_include_directories (csparse_extension PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${g2o_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  PRIVATE ${g2o_BINARY_DIR}
)

SET_TARGET_PROPERTIES(csparse_extension PROPERTIES OUTPUT_NAME ${LIB_PREFIX}csparse_extension)

IF(BUILD_CSPARSE)
  TARGET_LINK_LIBRARIES(csparse_extension PUBLIC csparse)
ELSE()
  TARGET_LINK_LIBRARIES(csparse_extension PUBLIC CXSparse::CXSparse)
ENDIF()

ADD_LIBRARY(solver_csparse
  solver_csparse.cpp
  linear_solver_csparse.h
)

generate_export_header (solver_csparse EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_solver_csparse_api.h
  EXPORT_MACRO_NAME G2O_SOLVER_CSPARSE_API)

target_include_directories (solver_csparse PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${g2o_BINARY_DIR}>
  PRIVATE ${g2o_BINARY_DIR}
)

SET_TARGET_PROPERTIES(solver_csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_csparse)

TARGET_LINK_LIBRARIES(solver_csparse csparse_extension core)

INSTALL(TARGETS solver_csparse csparse_extension EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers}
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_csparse_extension_api.h
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_solver_csparse_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/csparse)
