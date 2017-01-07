ADD_LIBRARY(calibration_odom_laser_library
  gm2dl_io.cpp gm2dl_io.h
  sclam_helpers.cpp sclam_helpers.h
  motion_information.h
  edge_se2_pure_calib.cpp edge_se2_pure_calib.h
  closed_form_calibration.cpp closed_form_calibration.h
)

generate_export_header (calibration_odom_laser_library PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h
  EXPORT_MACRO_NAME CALIBRATION_ODOM_LASER_API)

target_include_directories (calibration_odom_laser_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

SET_TARGET_PROPERTIES(calibration_odom_laser_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}calibration_odom_laser)
TARGET_LINK_LIBRARIES(calibration_odom_laser_library core solver_csparse
  types_slam2d types_sclam2d types_data)

ADD_EXECUTABLE(sclam_odom_laser
  sclam_odom_laser.cpp
)
SET_TARGET_PROPERTIES(sclam_odom_laser PROPERTIES OUTPUT_NAME sclam_odom_laser${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(sclam_odom_laser calibration_odom_laser_library)

ADD_EXECUTABLE(sclam_pure_calibration
  sclam_pure_calibration.cpp
)
SET_TARGET_PROPERTIES(sclam_pure_calibration PROPERTIES OUTPUT_NAME sclam_pure_calibration${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(sclam_pure_calibration calibration_odom_laser_library)

ADD_EXECUTABLE(sclam_laser_calib
  sclam_laser_calib.cpp
)
SET_TARGET_PROPERTIES(sclam_laser_calib PROPERTIES OUTPUT_NAME sclam_laser_calib${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(sclam_laser_calib calibration_odom_laser_library)
