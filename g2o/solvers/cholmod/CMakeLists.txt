ADD_LIBRARY(solver_cholmod
  solver_cholmod.cpp
  linear_solver_cholmod.h
)

target_include_directories (solver_cholmod PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_cholmod PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_cholmod)

TARGET_LINK_LIBRARIES(solver_cholmod SuiteSparse::CHOLMOD core)

INSTALL(TARGETS solver_cholmod EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/cholmod)
