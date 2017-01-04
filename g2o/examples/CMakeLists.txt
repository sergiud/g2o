ADD_SUBDIRECTORY(data_fitting)

ADD_SUBDIRECTORY(sphere)

# The condition on cholmod is required here because the cholmod solver
# is explicitly used in these examples.
IF(TARGET solver_cholmod)
  ADD_SUBDIRECTORY(target)
  ADD_SUBDIRECTORY(ba)
  ADD_SUBDIRECTORY(ba_anchored_inverse_depth)
ENDIF(TARGET solver_cholmod)

# Examples which explicitly use CSparse
IF(TARGET solver_csparse)
  ADD_SUBDIRECTORY(tutorial_slam2d)
  ADD_SUBDIRECTORY(icp)
  ADD_SUBDIRECTORY(calibration_odom_laser)
  ADD_SUBDIRECTORY(simple_optimize)
  ADD_SUBDIRECTORY(plane_slam)
  ADD_SUBDIRECTORY(line_slam)
ENDIF()

IF(TARGET solver_cholmod AND TARGET solver_csparse)
  ADD_SUBDIRECTORY(sba)
  ADD_SUBDIRECTORY(bal)
ENDIF()

IF(QGLVIEWER_FOUND AND TARGET solver_csparse)
  ADD_SUBDIRECTORY(slam2d)
ENDIF()

ADD_SUBDIRECTORY(data_convert)
ADD_SUBDIRECTORY(interactive_slam)
