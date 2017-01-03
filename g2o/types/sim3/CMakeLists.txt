ADD_LIBRARY(types_sim3
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

target_include_directories (types_sim3 PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

TARGET_LINK_LIBRARIES(types_sim3 types_sba)

generate_export_header (types_sim3 EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sim3_api.h
  EXPORT_MACRO_NAME G2O_TYPES_SIM3_API)

INSTALL(TARGETS types_sim3 EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers}
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_types_sim3_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/types/sim3)

