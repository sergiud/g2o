# download and compile gtest library

macro (create_test target)
  target_link_libraries(${target} GTest::gtest)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

ADD_SUBDIRECTORY(slam2d)
ADD_SUBDIRECTORY(slam3d)
ADD_SUBDIRECTORY(general)
