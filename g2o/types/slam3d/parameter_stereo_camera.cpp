// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/parameter_stereo_camera.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

G2O_START_NAMESPACE

  ParameterStereoCamera::ParameterStereoCamera(){
    setBaseline(0.075);
  }

  bool ParameterStereoCamera::read(std::istream& is) {
    Vector7d off;
    for (int i=0; i<7; i++)
      is >> off[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4D::MapType(off.data()+3).normalize();
    setOffset(internal::fromVectorQT(off));
    double fx,fy,cx,cy;
    is >> fx >> fy >> cx >> cy;
    setKcam(fx,fy,cx,cy);
    double baseline_;
    is >> baseline_;
    setBaseline(baseline_);
    return is.good();
  }
  
  bool ParameterStereoCamera::write(std::ostream& os) const {
    Vector7d off = internal::toVectorQT(_offset);
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    os << _Kcam(0,0) << " ";
    os << _Kcam(1,1) << " ";
    os << _Kcam(0,2) << " ";
    os << _Kcam(1,2) << " ";
    os << baseline() << " ";
    return os.good();
  }


G2O_END_NAMESPACE
