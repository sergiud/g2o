ADD_LIBRARY(solver_slam2d_linear
  slam2d_linear.cpp
  solver_slam2d_linear.h solver_slam2d_linear.cpp
)

generate_export_header (solver_slam2d_linear PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/solvers/slam2d_linear/g2o_slam2d_linear_api.h
  EXPORT_MACRO_NAME SLAM2D_LINEAR_API)

target_include_directories (solver_slam2d_linear PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(solver_slam2d_linear PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_slam2d_linear)
TARGET_LINK_LIBRARIES(solver_slam2d_linear solver_csparse types_slam2d)

INSTALL(TARGETS solver_slam2d_linear EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_slam2d_linear_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/solvers/slam2d_linear)
