# Need to find both Qt{4,5} and QGLViewer if the QQL support is to be built
FIND_PACKAGE(Qt5 COMPONENTS Core Xml OpenGL Gui Widgets)

IF(NOT Qt5_FOUND)
  FIND_PACKAGE(Qt4 COMPONENTS Core Xml OpenGL Gui Widgets)
  IF(NOT Qt4_FOUND AND NOT Qt5_FOUND)
    return ()
  ENDIF()
ENDIF()

FIND_PATH(QGLVIEWER_INCLUDE_DIR qglviewer.h
  /usr/include/QGLViewer
  /opt/local/include/QGLViewer
  /usr/local/include/QGLViewer
  /sw/include/QGLViewer
  ENV QGLVIEWERROOT
)

if (Qt5_FOUND)
  set (_QGLVIEWER_SUFFIX qt5)
elseif (Qt4_FOUND)
  set (_QGLVIEWER_SUFFIX qt4)
endif (Qt5_FOUND)

find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES qglviewer-${_QGLVIEWER_SUFFIX} qglviewer QGLViewer QGLViewer2
  QGLViewer-${_QGLVIEWER_SUFFIX}
  ENV QGLVIEWERROOT
  PATH_SUFFIXES QGLViewer QGLViewer/release
)
find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES qglviewer-${_QGLVIEWER_SUFFIX} dqglviewer dQGLViewer dQGLViewer2
  QGLViewerd2 QGLViewer-${_QGLVIEWER_SUFFIX}
  PATHS ENV QGLVIEWERROOT
  PATH_SUFFIXES QGLViewer QGLViewer/debug
)

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QGLViewer DEFAULT_MSG
  QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)
