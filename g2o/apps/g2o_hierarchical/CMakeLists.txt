ADD_LIBRARY(g2o_hierarchical_library
  edge_labeler.cpp
  edge_creator.cpp
  star.cpp
  edge_types_cost_function.cpp
  backbone_tree_action.cpp
  simple_star_ops.cpp
  g2o_hierarchical_test_functions.cpp
  edge_labeler.h
  edge_creator.h
  star.h
  edge_types_cost_function.h
  backbone_tree_action.h
  simple_star_ops.h
  g2o_hierarchical_api.h
)

generate_export_header (g2o_hierarchical_library PREFIX_NAME G2O_ EXPORT_FILE_NAME 
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_hierarchical_api.h
  EXPORT_MACRO_NAME HIERARCHICAL_API)

target_include_directories (g2o_hierarchical_library PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

set_target_properties (g2o_hierarchical_library PROPERTIES EXPORT_NAME hierarchical)

SET_TARGET_PROPERTIES(g2o_hierarchical_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}hierarchical)

TARGET_LINK_LIBRARIES(g2o_hierarchical_library core g2o_cli_library)

ADD_EXECUTABLE(g2o_hierarchical_application
  g2o_hierarchical.cpp)

TARGET_LINK_LIBRARIES(g2o_hierarchical_application g2o_hierarchical_library g2o_cli_library types_slam3d ) #types_slam3d_new

SET_TARGET_PROPERTIES(g2o_hierarchical_application PROPERTIES OUTPUT_NAME g2o_hierarchical${EXE_POSTFIX})

INSTALL(TARGETS g2o_hierarchical_library g2o_hierarchical_application EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES
  ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_hierarchical_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/apps/g2o_hierarchical)
