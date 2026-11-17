// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STEREO_CAMERA_PARAMETERS_H_
#define G2O_STEREO_CAMERA_PARAMETERS_H_

#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <g2o/types/slam3d/parameter_camera.h>

G2O_START_NAMESPACE
/**
 * \brief parameters for a camera
 */
class G2O_TYPES_SLAM3D_API ParameterStereoCamera : public ParameterCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ParameterStereoCamera();

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  void setBaseline(double baseline_) { _baseline = baseline_; }
  double baseline() const { return _baseline; }

 protected:
  double _baseline{};
};
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
