IF(TARGET solver_cholmod OR TARGET solver_csparse)
  ADD_EXECUTABLE(sba_demo sba_demo.cpp)
  TARGET_LINK_LIBRARIES(sba_demo core types_icp types_sba)
  SET_TARGET_PROPERTIES(sba_demo PROPERTIES OUTPUT_NAME sba_demo${EXE_POSTFIX})
ENDIF()

# Link CHOLMOD if available, CSparse otherwise
IF(TARGET solver_cholmod)
  TARGET_LINK_LIBRARIES(sba_demo solver_cholmod)
ELSEIF(TARGET solver_csparse)
  TARGET_LINK_LIBRARIES(sba_demo solver_csparse)
ENDIF()
