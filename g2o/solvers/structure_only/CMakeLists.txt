ADD_LIBRARY(solver_structure_only
  structure_only.cpp
  structure_only_solver.h
)

target_include_directories (solver_structure_only PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_structure_only PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_structure_only)
TARGET_LINK_LIBRARIES(solver_structure_only core)

INSTALL(TARGETS solver_structure_only EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/structure_only)
