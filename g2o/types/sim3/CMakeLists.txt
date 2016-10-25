ADD_LIBRARY(types_sim3
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

target_include_directories (types_sim3 PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

TARGET_LINK_LIBRARIES(types_sim3 types_sba)

generate_export_header (types_sim3 EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sim3_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SIM3_API)

INSTALL(TARGETS types_sim3 EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers}
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sim3_api.h
  DESTINATION include/g2o/types/sim3)

