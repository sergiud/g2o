ADD_LIBRARY(stuff
  filesys_tools.h    string_tools.h
  color_macros.h     macros.h         timeutil.cpp
  command_args.cpp   misc.h           sparse_helper.cpp  timeutil.h
  command_args.h     os_specific.c    sparse_helper.h
  filesys_tools.cpp  os_specific.h    string_tools.cpp
  property.cpp       property.h
  sampler.cpp        sampler.h        unscented.h
  tictoc.cpp tictoc.h
  g2o_stuff_api.h
)

if (TARGET Eigen3::Eigen)
  target_link_libraries (stuff PUBLIC Eigen3::Eigen)
else (TARGET Eigen3::Eigen)
  target_include_directories (stuff SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
endif (TARGET Eigen3::Eigen)

generate_export_header (stuff PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_stuff_api.h
  EXPORT_MACRO_NAME STUFF_API)

target_include_directories (stuff PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

SET_TARGET_PROPERTIES(stuff PROPERTIES OUTPUT_NAME ${LIB_PREFIX}stuff)

INSTALL(TARGETS stuff EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

# build our OpenGL helper library
IF(OPENGL_FOUND AND G2O_HAVE_OPENGL)
  ADD_LIBRARY(opengl_helper ${G2O_LIB_TYPE}
    opengl_primitives.cpp opengl_primitives.h
  )
  INSTALL(TARGETS opengl_helper EXPORT g2o-targets
    RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
    LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
    ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
  )
  TARGET_LINK_LIBRARIES(opengl_helper ${OPENGL_LIBRARIES})
  SET_TARGET_PROPERTIES(opengl_helper PROPERTIES OUTPUT_NAME ${LIB_PREFIX}opengl_helper)
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND NOT ANDROID)
  TARGET_LINK_LIBRARIES(stuff rt)
ENDIF()

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} ${CMAKE_CURRENT_BINARY_DIR}/g2o_stuff_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/stuff)
