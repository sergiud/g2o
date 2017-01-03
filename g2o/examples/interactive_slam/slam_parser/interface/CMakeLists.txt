SET(interface_library_src
  abstract_slam_interface.h
  parser_interface.cpp
  parser_interface.h
  slam_context_interface.cpp
  slam_context_interface.h
)

IF (MSVC)
  ADD_LIBRARY(interface_library STATIC
    ${interface_library_src}
  )
ELSE()
  ADD_LIBRARY(interface_library
    ${interface_library_src}
  )
ENDIF()

generate_export_header (interface_library EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/slam_parser_interface_api.h
  EXPORT_MACRO_NAME SLAM_PARSER_INTERFACE_API)

target_include_directories (interface_library PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

set_target_properties (interface_library PROPERTIES EXPORT_NAME interface)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../)

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
