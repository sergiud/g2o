ADD_LIBRARY(g2o_incremental_library
  graph_optimizer_sparse_incremental.cpp graph_optimizer_sparse_incremental.h
)

generate_export_header (g2o_incremental_library PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/examples/interactive_slam/g2o_incremental_api.h
  EXPORT_MACRO_NAME INCREMENTAL_API)

target_include_directories (g2o_incremental_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(g2o_incremental_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}incremental)
TARGET_LINK_LIBRARIES(g2o_incremental_library g2o_interactive_library solver_cholmod ${CAMD_LIBRARY})

ADD_EXECUTABLE(g2o_incremental_application g2o_incremental.cpp)
TARGET_LINK_LIBRARIES(g2o_incremental_application g2o_incremental_library)
SET_TARGET_PROPERTIES(g2o_incremental_application PROPERTIES OUTPUT_NAME g2o_incremental${EXE_POSTFIX})

INSTALL(TARGETS g2o_incremental_library g2o_incremental_application
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${_g2o_CMake_INCLUDEDIR}/examples/interactive_slam/g2o_incremental)
