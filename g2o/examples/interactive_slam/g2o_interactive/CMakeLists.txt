ADD_LIBRARY(g2o_interactive_library
  g2o_slam_interface.cpp g2o_slam_interface.h
  graph_optimizer_sparse_online.cpp graph_optimizer_sparse_online.h
  types_online.cpp
)

generate_export_header (g2o_interactive_library PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_interactive_api.h
  EXPORT_MACRO_NAME INTERACTIVE_API)

target_include_directories (g2o_interactive_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(g2o_interactive_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}interactive)
TARGET_LINK_LIBRARIES(g2o_interactive_library core types_slam2d types_slam3d solver_cholmod parser_library interface_library)

ADD_EXECUTABLE(g2o_online_application g2o_online.cpp)
TARGET_LINK_LIBRARIES(g2o_online_application g2o_interactive_library)
SET_TARGET_PROPERTIES(g2o_online_application PROPERTIES OUTPUT_NAME g2o_online${EXE_POSTFIX})

#ADD_EXECUTABLE(generate_commands_application generate_commands.cpp)
#TARGET_LINK_LIBRARIES(generate_commands_application g2o_interactive_library)
#SET_TARGET_PROPERTIES(generate_commands_application PROPERTIES OUTPUT_NAME generate_commands)

INSTALL(TARGETS g2o_interactive_library g2o_online_application
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${_g2o_CMake_INCLUDEDIR}/examples/interactive_slam/g2o_interactive)
