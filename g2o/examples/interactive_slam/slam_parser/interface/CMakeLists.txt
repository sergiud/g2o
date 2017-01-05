SET(interface_library_src
  abstract_slam_interface.h
  parser_interface.cpp
  parser_interface.h
  slam_context_interface.cpp
  slam_context_interface.h
)

ADD_LIBRARY(interface_library
  ${interface_library_src}
)

generate_export_header (interface_library PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_slam_parser_interface_api.h
  EXPORT_MACRO_NAME SLAM_PARSER_INTERFACE_API)

target_include_directories (interface_library PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_link_libraries (interface_library parser_library)

set_target_properties (interface_library PROPERTIES EXPORT_NAME interface)

SET_TARGET_PROPERTIES(interface_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}interface)
TARGET_LINK_LIBRARIES(interface_library parser_library)

INSTALL(TARGETS interface_library EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/slam_parser_interface_api.h
  DESTINATION include/slam_parser/interface)
