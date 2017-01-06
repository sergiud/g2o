ADD_LIBRARY(types_icp
  types_icp.cpp  types_icp.h
)

generate_export_header (types_icp PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/types/icp/g2o_types_icp_api.h
  EXPORT_MACRO_NAME TYPES_ICP_API)

target_include_directories (types_icp PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(types_icp PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_icp)

TARGET_LINK_LIBRARIES(types_icp types_sba types_slam3d)

INSTALL(TARGETS types_icp EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} ${CMAKE_BINARY_DIR}/include/g2o/types/icp/g2o_types_icp_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/icp)
