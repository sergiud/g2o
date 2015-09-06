ADD_LIBRARY(types_sim3
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

target_include_directories (types_sim3 PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
)

SET_TARGET_PROPERTIES(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

TARGET_LINK_LIBRARIES(types_sim3 types_sba)

INSTALL(TARGETS types_sim3 EXPORT g2o-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/sim3)

