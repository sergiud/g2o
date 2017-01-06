ADD_LIBRARY(types_data
  types_data.cpp
  robot_data.cpp robot_data.h
  vertex_tag.cpp vertex_tag.h
  vertex_ellipse.cpp vertex_ellipse.h
  laser_parameters.cpp laser_parameters.h
  raw_laser.cpp raw_laser.h
  robot_laser.cpp robot_laser.h
  data_queue.cpp data_queue.h
  g2o_types_data_api.h
)

generate_export_header (types_data PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/types/data/g2o_types_data_api.h
  EXPORT_MACRO_NAME TYPES_DATA_API)

target_include_directories (types_data PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(types_data PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_data)

TARGET_LINK_LIBRARIES(types_data core types_slam2d)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_data freeglut_minimal opengl_helper)
ENDIF()

INSTALL(TARGETS types_data EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_data_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/data)
