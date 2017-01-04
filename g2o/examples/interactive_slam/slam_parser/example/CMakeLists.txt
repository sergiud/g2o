ADD_LIBRARY(example_library
  example_slam_interface.cpp
  example_slam_interface.h
)

generate_export_header (example_library EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_example_api.h
  EXPORT_MACRO_NAME G2O_EXAMPLE_API)

target_include_directories (example_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(example_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}example)

TARGET_LINK_LIBRARIES(example_library parser_library interface_library)

ADD_EXECUTABLE(test_slam_interface test_slam_interface.cpp)
TARGET_LINK_LIBRARIES(test_slam_interface example_library)
SET_TARGET_PROPERTIES(test_slam_interface PROPERTIES OUTPUT_NAME test_slam_interface${EXE_POSTFIX})

#INSTALL(TARGETS example_library
  #RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  #LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  #ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
#)

#FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
#INSTALL(FILES ${headers} DESTINATION include/slam_parser/example)
