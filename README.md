<!--
SPDX-FileCopyrightText: 2015 Rainer Kuemmerle, Giorgio Grisetti
SPDX-FileCopyrightText: 2026 Sergiu Deitsch

SPDX-License-Identifier: BSD-2-Clause
-->

# g2o

g2o optimizes nonlinear error functions that can be represented as a graph,
such as in simultaneous localization and mapping (SLAM) and bundle adjustment.
A new problem takes a few lines of code: a vertex for each state variable and
an edge for each measurement. Rainer Kümmerle, Giorgio Grisetti, Hauke
Strasdat, Kurt Konolige and Wolfram Burgard created it [1].

## Motivation

This is a fork of the [original g2o](https://github.com/RainerKuemmerle/g2o)
that keeps the interface of version 2017.7.30. The g2o releases since then
changed that interface, while projects such as
[this fork of ORB-SLAM](https://github.com/sergiud/ORB_SLAM) still use it.

The fork builds as a single CMake 3.28 project that installs a CMake package
and a software bill of materials, with Eigen up to version 5, Qt 6 and
SuiteSparse 7, and logs through ng-log. It fixes defects of the original, such
as installed headers that do not compile, exports missing from shared
libraries, the alignment of Eigen types and a crash on exit of programs that
draw with OpenGL.

## Requirements

* CMake 3.28 and a C++20 compiler
* Eigen 3.4.1 up to version 5
* [ng-log](https://github.com/ng-log/ng-log) 0.9.0 with its experimental C++20
  interface, which only its development branch provides yet

The following are optional:

* SuiteSparse 7, whose CHOLMOD and CXSparse libraries the sparse solvers use
* OpenGL with GLU to draw graphs, turned on with `-DWITH_OPENGL=ON`
* Qt 6 and libQGLViewer built for Qt 6, which the graph viewer `g2o_viewer`
  needs together with OpenGL
* GoogleTest 1.16 for the unit tests

The build leaves out the parts whose optional dependencies are missing.

## Getting Started

Build g2o and run its tests:

```sh
git clone -b cmake https://github.com/sergiud/g2o.git
cd g2o
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Add the installation prefix of a dependency that CMake does not find, such as
ng-log, with `-DCMAKE_PREFIX_PATH=<prefix>`. Then generate a pose graph on a
sphere and optimize it with 10 iterations:

```sh
build/bin/create_sphere -o sphere.g2o
build/bin/g2o -i 10 -o optimized.g2o sphere.g2o
```

The programs write their messages to the standard error. `-h` lists the
options of the command line tools, and `g2o -listSolvers` the available
solvers.

## Advanced Usage

These CMake options select what is built:

| Option | Default | Effect |
| --- | --- | --- |
| `BUILD_APPS` | `ON` | the command line tools and the viewer |
| `BUILD_EXAMPLES` | `ON` | the examples |
| `BUILD_SHARED_LIBS` | `ON` | shared libraries, which the plugins of `g2o` need |
| `BUILD_LGPL_SHARED_LIBS` | `OFF` | the LGPL libraries as shared libraries |
| `BUILD_LINKED_APPS` | `OFF` | variants of the tools linked to the solvers and types instead of loading them |
| `WITH_OPENGL` | `OFF` | drawing with OpenGL |
| `WITH_OPENMP` | `OFF` | experimental parallelization with OpenMP |
| `WITH_SBOM` | `ON` | a software bill of materials in SPDX format, which CMake 4.5 generates, 4.3 and 4.4 generate experimentally and earlier versions skip |

`cmake --install build --prefix <prefix>` installs the programs into `bin`,
the libraries and the CMake package into `lib/g2o`, the headers into
`include` and the bill of materials into `share/sbom`. The installed programs
and libraries find the libraries of g2o relative to their own location, so
that the installation can be moved. Dependencies installed elsewhere, such as
ng-log, must be on the search path of the dynamic linker. Other projects link
against the libraries they use:

```cmake
find_package (g2o 2017 REQUIRED COMPONENTS core types_slam3d)

target_link_libraries (app PRIVATE g2o::core g2o::types_slam3d)
```

The components are the names of the libraries, such as `stuff`, `core`,
`solver_cholmod`, `solver_csparse`, `types_slam2d`, `types_slam3d`,
`types_sba` and `types_sim3`.

The libraries report warnings and errors through the experimental logging
functions of ng-log and leave it to the application where the messages go.
Without any setup, ng-log writes them to the standard error after a warning
that logging was not initialized. `g2o::initializeLogging(argv[0])` from
`g2o/stuff/logger.h` of `g2o::stuff`, which `g2o::core` links, sends them to the standard error only, as the programs
of g2o do, and the setup functions of ng-log send them to log files or sinks.

`doc/g2o.pdf` describes the structure of the library and how to extend it,
and `doc/doxygen/readme.txt` how to generate the API documentation.

## Licensing

g2o is released under the [BSD 2-Clause License](LICENSES/BSD-2-Clause.txt).
Some parts are released under the LGPL version 3 or later or the GPL version 3
or later, such as
the CSparse solver under the LGPL and the graph viewer, the incremental SLAM
example and the 2D SLAM example under the GPL, as the SPDX header of each file
states. The included code of other projects keeps its license: the automatic
differentiation of Ceres under the BSD 3-Clause License and the font rendering
of freeglut under the MIT License. [LICENSES](LICENSES) holds the full text of
each license.

CHOLMOD contains features under the GPL, which the builds of Debian and Ubuntu
include. A program that links the CHOLMOD solver of g2o to such a build falls
under the GPL, unless CHOLMOD is rebuilt without them.

If you use g2o in academic work, please cite [1]:

1. Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige and
   Wolfram Burgard. g2o: A General Framework for Graph Optimization. *IEEE
   International Conference on Robotics and Automation (ICRA)*, 3607–3613,
   2011. [doi:10.1109/ICRA.2011.5979949](https://doi.org/10.1109/ICRA.2011.5979949)
