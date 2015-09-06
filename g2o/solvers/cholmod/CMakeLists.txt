ADD_LIBRARY(solver_cholmod
  solver_cholmod.cpp
  linear_solver_cholmod.h
)

target_include_directories (solver_cholmod PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_cholmod PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_cholmod)

ADD_DEFINITIONS(${BLAS_DEFINITIONS} ${LAPACK_DEFINITIONS})
TARGET_LINK_LIBRARIES(solver_cholmod SuiteSparse::CHOLMOD core)

INSTALL(TARGETS solver_cholmod EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/solvers/cholmod)
