SET(G2O_PARSER_SRC
  slam_context.h slam_context.cpp
  bison_parser.cpp bison_parser.h
  commands.h
  driver.h driver.cpp
  location.hh
  position.hh
  scanner.h flex_scanner.cpp
  stack.hh
)

ADD_LIBRARY(parser_library
  ${G2O_PARSER_SRC}
)

generate_export_header (parser_library PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/slam_parser/g2o_slam_parser_api.h
  EXPORT_MACRO_NAME SLAM_PARSER_API)

target_include_directories (parser_library PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

set_target_properties (parser_library PROPERTIES EXPORT_NAME parser)

SET_TARGET_PROPERTIES(parser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}parser)

ADD_EXECUTABLE(test_slam_parser test_slam_parser.cpp)
TARGET_LINK_LIBRARIES(test_slam_parser parser_library)
SET_TARGET_PROPERTIES(test_slam_parser PROPERTIES OUTPUT_NAME test_slam_parser${EXE_POSTFIX})

INSTALL(TARGETS parser_library EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/slam_parser_api.h
  DESTINATION include/g2o/slam_parser/parser)
