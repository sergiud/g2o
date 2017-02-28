ADD_LIBRARY(freeglut_minimal
  freeglut_font.cpp
  freeglut_stroke_mono_roman.cpp
  freeglut_stroke_roman.cpp
)

generate_export_header (freeglut_minimal PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/freeglut/g2o_freeglut_api.h
  EXPORT_MACRO_NAME FGAPI)


target_include_directories (freeglut_minimal PUBLIC ${OPENGL_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(freeglut_minimal ${OPENGL_LIBRARIES} stuff)

SET_TARGET_PROPERTIES(freeglut_minimal PROPERTIES OUTPUT_NAME ${LIB_PREFIX}ext_freeglut_minimal)

INSTALL(TARGETS freeglut_minimal EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers}
  ${CMAKE_BINARY_DIR}/include/g2o/freeglut/g2o_freeglut_api.h
  DESTINATION include/g2o/freeglut)
