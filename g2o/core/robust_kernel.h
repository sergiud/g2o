// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2015-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_ROBUST_KERNEL_H
#define G2O_ROBUST_KERNEL_H

#include <Eigen/Core>
#include <memory>

#include <g2o/config.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/g2o_core_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief base for all robust cost functions
   *
   * Note in all the implementations for the other cost functions the e in the
   * funtions corresponds to the sqaured errors, i.e., the robustification
   * functions gets passed the squared error.
   *
   * e.g. the robustified least squares function is
   *
   * chi^2 = sum_{e} rho( e^T Omega e )
   */
  class G2O_CORE_API RobustKernel
  {
    public:
      RobustKernel();
      explicit RobustKernel(double delta);
      virtual ~RobustKernel() = default;
      /**
       * compute the scaling factor for a error:
       * The error is e^T Omega e
       * The output rho is
       * rho[0]: The actual scaled error value
       * rho[1]: First derivative of the scaling function
       * rho[2]: Second derivative of the scaling function
       */
      virtual void robustify(double squaredError, Vector3D& rho) const = 0;

      /**
       * set the window size of the error. A squared error above delta^2 is considered
       * as outlier in the data.
       */
      virtual void setDelta(double delta);
      double delta() const { return _delta;}

    protected:
      double _delta;
  };
  typedef std::unique_ptr<RobustKernel> RobustKernelPtr;

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
