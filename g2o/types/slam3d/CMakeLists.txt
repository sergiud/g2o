ADD_LIBRARY(types_slam3d
  dquat2mat.cpp dquat2mat.h
  isometry3d_mappings.h
  isometry3d_mappings.cpp
  isometry3d_gradients.cpp
  isometry3d_gradients.h
  vertex_se3.cpp
  vertex_se3.h
  parameter_se3_offset.cpp
  parameter_se3_offset.h
  edge_se3.cpp
  edge_se3.h
  edge_se3_offset.cpp
  edge_se3_offset.h
  vertex_pointxyz.cpp
  vertex_pointxyz.h
  parameter_camera.cpp
  parameter_camera.h
  parameter_stereo_camera.cpp
  parameter_stereo_camera.h
  edge_se3_pointxyz.cpp
  edge_se3_pointxyz.h
  edge_se3_pointxyz_disparity.cpp
  edge_se3_pointxyz_disparity.h
  edge_se3_pointxyz_depth.cpp
  edge_se3_pointxyz_depth.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  edge_se3_lotsofxyz.cpp
  edge_se3_lotsofxyz.h
  se3quat.h
  se3_ops.h se3_ops.hpp
  edge_pointxyz.cpp edge_pointxyz.h
  types_slam3d.cpp
  types_slam3d.h
)

generate_export_header (types_slam3d PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/types/slam3d/g2o_types_slam3d_api.h
  EXPORT_MACRO_NAME TYPES_SLAM3D_API)

target_include_directories (types_slam3d PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)
TARGET_LINK_LIBRARIES(types_slam3d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam3d opengl_helper ${OPENGL_gl_LIBRARY} )
ENDIF()

INSTALL(TARGETS types_slam3d EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_slam3d_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/slam3d)
