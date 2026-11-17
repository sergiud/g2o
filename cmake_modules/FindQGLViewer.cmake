# SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
# SPDX-FileCopyrightText: 2016-2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

#[=======================================================================[.rst:
FindQGLViewer
-------------

Finds the libQGLViewer library built for Qt 6.

Imported targets
^^^^^^^^^^^^^^^^

``QGLViewer::QGLViewer``
  The libQGLViewer library, if found.

Result variables
^^^^^^^^^^^^^^^^

``QGLViewer_FOUND``
  True if the headers and the library were found.

Cache variables
^^^^^^^^^^^^^^^

``QGLViewer_INCLUDE_DIR``
  The directory containing ``qglviewer.h``.
``QGLViewer_LIBRARY_RELEASE``, ``QGLViewer_LIBRARY_DEBUG``
  The release and the debug library.
#]=======================================================================]

include (FindPackageHandleStandardArgs)
include (SelectLibraryConfigurations)

find_package (Qt6 QUIET COMPONENTS Core Gui OpenGLWidgets Widgets Xml)

find_path (QGLViewer_INCLUDE_DIR qglviewer.h
  PATH_SUFFIXES QGLViewer
  PATHS ENV QGLVIEWERROOT
)

# Distributions that build libQGLViewer for several versions of Qt name the
# libraries after the version, others name the library of the only version
# they build without it.
find_library (QGLViewer_LIBRARY_RELEASE
  NAMES QGLViewer-qt6 qglviewer-qt6 QGLViewer qglviewer
  PATHS ENV QGLVIEWERROOT
  PATH_SUFFIXES QGLViewer QGLViewer/release
)
find_library (QGLViewer_LIBRARY_DEBUG
  NAMES QGLViewer-qt6d qglviewer-qt6d QGLViewerd dqglviewer
  PATHS ENV QGLVIEWERROOT
  PATH_SUFFIXES QGLViewer QGLViewer/debug
)

select_library_configurations (QGLViewer)

mark_as_advanced (
  QGLViewer_INCLUDE_DIR
  QGLViewer_LIBRARY_DEBUG
  QGLViewer_LIBRARY_RELEASE
)

# The library is usable only together with the Qt it was built for.
set (_QGLViewer_HAVE_QT FALSE)

if (TARGET Qt6::OpenGLWidgets AND TARGET Qt6::Xml)
  set (_QGLViewer_HAVE_QT TRUE)
endif (TARGET Qt6::OpenGLWidgets AND TARGET Qt6::Xml)

find_package_handle_standard_args (QGLViewer
  REQUIRED_VARS QGLViewer_INCLUDE_DIR QGLViewer_LIBRARY _QGLViewer_HAVE_QT
)

unset (_QGLViewer_HAVE_QT)

if (QGLViewer_FOUND AND NOT TARGET QGLViewer::QGLViewer)
  add_library (QGLViewer::QGLViewer UNKNOWN IMPORTED)
  set_target_properties (QGLViewer::QGLViewer PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${QGLViewer_INCLUDE_DIR}
    INTERFACE_LINK_LIBRARIES
      "Qt6::Core;Qt6::Gui;Qt6::OpenGLWidgets;Qt6::Widgets;Qt6::Xml"
  )

  if (QGLViewer_LIBRARY_RELEASE)
    set_property (TARGET QGLViewer::QGLViewer APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE
    )
    set_target_properties (QGLViewer::QGLViewer PROPERTIES
      IMPORTED_LOCATION_RELEASE ${QGLViewer_LIBRARY_RELEASE}
    )
  endif (QGLViewer_LIBRARY_RELEASE)

  if (QGLViewer_LIBRARY_DEBUG)
    set_property (TARGET QGLViewer::QGLViewer APPEND PROPERTY
      IMPORTED_CONFIGURATIONS DEBUG
    )
    set_target_properties (QGLViewer::QGLViewer PROPERTIES
      IMPORTED_LOCATION_DEBUG ${QGLViewer_LIBRARY_DEBUG}
    )
  endif (QGLViewer_LIBRARY_DEBUG)
endif (QGLViewer_FOUND AND NOT TARGET QGLViewer::QGLViewer)
