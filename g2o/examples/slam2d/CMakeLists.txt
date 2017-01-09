QT5_WRAP_UI(UI_HEADERS base_main_window.ui)
QT5_WRAP_CPP(UI_SOURCES main_window.h)

ADD_EXECUTABLE(slam2d_g2o
  main_window.cpp
  slam2d_viewer.cpp
  slam2d_viewer.h
  slam2d_g2o.cpp
  draw_helpers.cpp
  ${UI_HEADERS}
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(slam2d_g2o PROPERTIES OUTPUT_NAME slam2d_g2o${EXE_POSTFIX})

INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

TARGET_LINK_LIBRARIES(slam2d_g2o core solver_csparse types_slam2d
  ${QGLVIEWER_LIBRARY} Qt5::Xml Qt5::OpenGL Qt5::Gui Qt5::Core
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
