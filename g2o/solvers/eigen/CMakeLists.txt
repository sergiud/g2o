ADD_LIBRARY(solver_eigen
  solver_eigen.cpp
  linear_solver_eigen.h
)

target_include_directories (solver_eigen PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_eigen PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_eigen)
TARGET_LINK_LIBRARIES(solver_eigen core)

INSTALL(TARGETS solver_eigen EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)
FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/eigen)
