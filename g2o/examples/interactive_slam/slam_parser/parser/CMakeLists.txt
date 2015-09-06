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

IF (MSVC)
ADD_LIBRARY(parser_library STATIC
  ${G2O_PARSER_SRC}
)
ELSE()
ADD_LIBRARY(parser_library
  ${G2O_PARSER_SRC}
)
ENDIF()

generate_export_header (parser_library EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/slam_parser_api.h
  EXPORT_MACRO_NAME SLAM_PARSER_API)

target_include_directories (parser_library PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

set_target_properties (parser_library PROPERTIES EXPORT_NAME parser)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET_TARGET_PROPERTIES(parser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}parser)

#ADD_EXECUTABLE(test_slam_parser test_slam_parser.cpp)
#TARGET_LINK_LIBRARIES(test_slam_parser parser_library)
#SET_TARGET_PROPERTIES(test_slam_parser PROPERTIES OUTPUT_NAME test_slam_parser${EXE_POSTFIX})

INSTALL(TARGETS parser_library EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh")
INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/slam_parser_api.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/slam_parser/parser)
