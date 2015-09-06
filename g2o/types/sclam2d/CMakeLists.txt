ADD_LIBRARY(types_sclam2d
  edge_se2_sensor_calib.cpp edge_se2_sensor_calib.h
  vertex_odom_differential_params.cpp vertex_odom_differential_params.h
  edge_se2_odom_differential_calib.cpp edge_se2_odom_differential_calib.h
  odometry_measurement.cpp odometry_measurement.h
  types_sclam2d.cpp types_sclam2d.h
)

generate_export_header (types_sclam2d EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sclam2d_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SCLAM2D_API)

target_include_directories (types_sclam2d PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_sclam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sclam2d)

TARGET_LINK_LIBRARIES(types_sclam2d types_slam2d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_sclam2d ${OPENGL_gl_LIBRARY})
ENDIF()

INSTALL(TARGETS types_sclam2d EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sclam2d_api.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/sclam2d)
