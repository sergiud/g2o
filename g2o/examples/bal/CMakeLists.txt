IF(TARGET solver_cholmod OR TARGET solver_csparse)
  ADD_EXECUTABLE(bal_example bal_example.cpp)
ENDIF()

# Link CHOLMOD if available, CSparse otherwise
IF(TARGET solver_cholmod)
  TARGET_LINK_LIBRARIES(bal_example solver_cholmod)
ELSEIF(TARGET solver_csparse)
  TARGET_LINK_LIBRARIES(bal_example solver_csparse)
ENDIF()
