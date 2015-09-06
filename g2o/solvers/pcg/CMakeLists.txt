ADD_LIBRARY(solver_pcg
  solver_pcg.cpp
  linear_solver_pcg.h  
  linear_solver_pcg.hpp
)

target_include_directories (solver_pcg PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(solver_pcg PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_pcg)

TARGET_LINK_LIBRARIES(solver_pcg core)

INSTALL(TARGETS solver_pcg EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/solvers/pcg)
