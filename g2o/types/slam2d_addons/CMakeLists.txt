ADD_LIBRARY(types_slam2d_addons
  line_2d.h
  types_slam2d_addons.cpp types_slam2d_addons.h
  vertex_segment2d.cpp         vertex_segment2d.h
  edge_se2_segment2d.cpp       edge_se2_segment2d.h
  edge_se2_segment2d_line.cpp       edge_se2_segment2d_line.h
  edge_se2_segment2d_pointLine.cpp       edge_se2_segment2d_pointLine.h
  vertex_line2d.cpp            vertex_line2d.h
  edge_se2_line2d.cpp          edge_se2_line2d.h
  edge_line2d_pointxy.cpp          edge_line2d_pointxy.h
  edge_line2d.cpp          edge_line2d.h
)

generate_export_header (types_slam2d_addons EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_slam2d_addons_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SLAM2D_ADDONS_API)

target_include_directories (types_slam2d_addons PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_slam2d_addons PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d_addons)

TARGET_LINK_LIBRARIES(types_slam2d_addons types_slam2d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam2d_addons ${OPENGL_gl_LIBRARY})
ENDIF()

INSTALL(TARGETS types_slam2d_addons EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_slam2d_addons_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/slam2d_addons)
