ADD_LIBRARY(types_icp
  types_icp.cpp  types_icp.h
)

generate_export_header (types_icp EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_icp_api.h
  EXPORT_MACRO_NAME G2O_TYPES_ICP_API)

target_include_directories (types_icp PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_icp PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_icp)

TARGET_LINK_LIBRARIES(types_icp types_sba types_slam3d)

INSTALL(TARGETS types_icp EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_icp_api.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/icp)
