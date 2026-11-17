g2o - General Graph Optimization
================================

Linux: [![Build Status](https://travis-ci.org/RainerKuemmerle/g2o.svg?branch=master)](https://travis-ci.org/RainerKuemmerle/g2o)
Windows: [![Build status](https://ci.appveyor.com/api/projects/status/9w0cpb9krc6t4nt7/branch/master?svg=true)](https://ci.appveyor.com/project/RainerKuemmerle/g2o/branch/master)

g2o is an open-source C++ framework for optimizing graph-based nonlinear error
functions. g2o has been designed to be easily extensible to a wide range of
problems and a new problem typically can be specified in a few lines of code.
The current implementation provides solutions to several variants of SLAM and
BA.

A wide range of problems in robotics as well as in computer-vision involve the
minimization of a non-linear error function that can be represented as a graph.
Typical instances are simultaneous localization and mapping (SLAM) or bundle
adjustment (BA). The overall goal in these problems is to find the
configuration of parameters or state variables that maximally explain a set of
measurements affected by Gaussian noise. g2o is an open-source C++ framework
for such nonlinear least squares problems. g2o has been designed to be easily
extensible to a wide range of problems and a new problem typically can be
specified in a few lines of code. The current implementation provides solutions
to several variants of SLAM and BA. g2o offers a performance comparable to
implementations of state-of-the-art approaches for the specific problems
(02/2011).

### Papers Describing the Approach:
Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat,
Kurt Konolige, and Wolfram Burgard
g2o: A General Framework for Graph Optimization
IEEE International Conference on Robotics and Automation (ICRA), 2011
http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf

### Documentation
A detailed description of how the library is structured and how to use and extend it can be found in /doc/g2o.pdf
The API documentation can be generated as described in doc/doxygen/readme.txt

### License
g2o is licensed under the BSD-2-Clause license. However, some libraries are
available under different license terms. See below. Every file states its
copyright holders and license with SPDX tags, or through REUSE.toml for files
that cannot carry them, following the REUSE specification.

The following parts are licensed under LGPL3+:
- csparse\_extension

The following parts are licensed under GPL3+:
- g2o\_viewer
- g2o\_incremental
- slam2d\_g2o (example for 2D SLAM with a QGLviewer GUI)

Please note that some features of CHOLMOD (which may be used by g2o, see
libsuitesparse below) are licensed under the GPL. To avoid that your binary has
to be licensed under the GPL, you may have to re-compile CHOLMOD without
including its GPL features. The CHOLMOD library distributed with, for example,
Ubuntu or Debian includes the GPL features. The supernodal factorization is
considered by g2o, if it is available.

Within the folder EXTERNAL we include software not written by us to
guarantee easy compilation.
- ceres: BSD (see EXTERNAL/ceres/LICENSE)
  Headers to perform Automatic Differentiation
- freeglut: MIT (Copyright (c) 1999-2000 Pawel W. Olszta)
  We use a stripped down version for drawing text in OpenGL.

The LICENSES folder holds the full text of each license.

g2o is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
licenses for more details.


### Requirements
  * cmake 3.28        http://www.cmake.org/
  * Eigen3            http://eigen.tuxfamily.org

  On Ubuntu / Debian these dependencies are resolved by installing the
  following packages.
    - cmake
    - libeigen3-dev

#### Optional requirements
  * suitesparse       http://www.cise.ufl.edu/research/sparse/SuiteSparse/
    whose CHOLMOD and CXSparse libraries the sparse solvers build on
  * Qt6               http://qt-project.org
  * libQGLViewer      http://www.libqglviewer.com/ built for Qt6

  On Ubuntu / Debian these dependencies are resolved by installing the
  following packages.
    - libsuitesparse-dev
    - qt6-base-dev
    - libqglviewer-dev-qt6

#### Mac OS X
If using [Homebrew](http://brew.sh/), then

`brew install homebrew/science/g2o`

will install g2o together with its required dependencies. In this case no manual compilation is necessary.

### Compilation
Our primary development platform is Linux. Experimental support for
Mac OS X, Android and Windows (MinGW or MSVC).
We recommend a so-called out of source build which can be achieved
by the following command sequence.

- `mkdir build`
- `cd build`
- `cmake ../`
- `make`

The binaries will be placed in bin and the libraries in lib which
are both located in the build folder.

With CMake 4.5 or later, and experimentally with CMake 4.3 and 4.4,
`cmake --install` also installs a software bill of materials in SPDX format to
`share/sbom`, unless the option `WITH_SBOM` is turned off.
If you are compiling on Windows, please download Eigen3 and extract it.
Within cmake-gui set the variable G2O\_EIGEN3\_INCLUDE to that directory.

### Cross-Compiling for Android

- `mkdir build`
- `cd build`
- `cmake -DCMAKE_TOOLCHAIN_FILE=../script/android.toolchain.cmake -DANDROID_NDK=<YOUR_PATH_TO_ANDROID_NDK_r10d+> -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON" -DEIGEN3_INCLUDE_DIR="<YOUR_PATH_TO_EIGEN>" -DEIGEN3_VERSION_OK=ON .. && cmake --build .`


### Acknowledgments
We thank the following contributors for providing patches:
- Simon J. Julier: patches to achieve compatibility with Mac OS X and others.
- Michael A. Eriksen for submitting patches to compile with MSVC.
- Mark Pupilli for submitting patches to compile with MSVC.


### Contact information
Rainer Kuemmerle <kuemmerl@informatik.uni-freiburg.de>   
Giorgio Grisetti <grisetti@dis.uniroma1.it>   
Hauke Strasdat <strasdat@gmail.com>   
Kurt Konolige <konolige@willowgarage.com>   
Wolfram Burgard <burgard@informatik.uni-freiburg.de>   
