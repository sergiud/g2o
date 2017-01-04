if (MSVC)
  add_compile_options (/bigobj)
endif (MSVC)

ADD_LIBRARY(types_slam3d_addons
  vertex_se3_euler.cpp
  vertex_se3_euler.h
  edge_se3_euler.cpp
  edge_se3_euler.h
  vertex_plane.cpp
  vertex_plane.h
  edge_se3_plane_calib.cpp
  edge_se3_plane_calib.h
  line3d.cpp line3d.h
  vertex_line3d.cpp vertex_line3d.h
  edge_se3_line.cpp edge_se3_line.h
  edge_plane.cpp edge_plane.h
  edge_se3_calib.cpp edge_se3_calib.h
  types_slam3d_addons.cpp
  types_slam3d_addons.h
)

generate_export_header (types_slam3d_addons EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_slam3d_addons_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SLAM3D_ADDONS_API)

target_include_directories (types_slam3d_addons PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_compile_definitions (types_slam3d_addons PRIVATE EIGEN_NO_STATIC_ASSERT)

SET_TARGET_PROPERTIES(types_slam3d_addons PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d_addons)

TARGET_LINK_LIBRARIES(types_slam3d_addons types_slam3d core)
IF(G2O_HAVE_OPENGL)
  TARGET_LINK_LIBRARIES(types_slam3d_addons ${OPENGL_gl_LIBRARY} )
ENDIF()

INSTALL(TARGETS types_slam3d_addons EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_slam3d_addons_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/slam3d_addons)
