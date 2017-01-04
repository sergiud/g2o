ADD_SUBDIRECTORY(slam_parser)

IF(TARGET solver_cholmod)
  ADD_SUBDIRECTORY(g2o_interactive)
  ADD_SUBDIRECTORY(g2o_incremental)
ENDIF(TARGET solver_cholmod)
