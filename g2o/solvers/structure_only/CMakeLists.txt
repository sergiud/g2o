ADD_LIBRARY(solver_structure_only
  structure_only.cpp
  structure_only_solver.h
)

target_include_directories (solver_structure_only PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_structure_only PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_structure_only)
TARGET_LINK_LIBRARIES(solver_structure_only core)

INSTALL(TARGETS solver_structure_only EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/solvers/structure_only)
