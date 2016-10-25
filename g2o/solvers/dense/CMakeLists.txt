ADD_LIBRARY(solver_dense
  solver_dense.cpp linear_solver_dense.h
)

target_include_directories (solver_dense PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_dense PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_dense)

TARGET_LINK_LIBRARIES(solver_dense core)

INSTALL(TARGETS solver_dense EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION include/g2o/solvers/dense)
