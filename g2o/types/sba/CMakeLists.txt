ADD_LIBRARY(types_sba
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
)

generate_export_header (types_sba EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sba_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SBA_API)

target_include_directories (types_sba PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)

TARGET_LINK_LIBRARIES(types_sba core types_slam3d)

INSTALL(TARGETS types_sba EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sba_api.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/sba)
