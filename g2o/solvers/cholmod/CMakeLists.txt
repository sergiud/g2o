ADD_LIBRARY(solver_cholmod
  solver_cholmod.cpp
  linear_solver_cholmod.h
  linear_solver_cholmod.cpp
)

generate_export_header (solver_cholmod PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/cholmod/g2o_solver_cholmod_api.h
  EXPORT_MACRO_NAME SOLVER_CHOLMOD_API)

target_include_directories (solver_cholmod PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(solver_cholmod PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_cholmod)

TARGET_LINK_LIBRARIES(solver_cholmod SuiteSparse::CHOLMOD core)

INSTALL(TARGETS solver_cholmod EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers}
  ${CMAKE_BINARY_DIR}/include/g2o/solvers/cholmod/g2o_solver_cholmod_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/cholmod)
