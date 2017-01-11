#include <g2o/core/base_edge.h>

G2O_START_NAMESPACE

  template class G2O_CORE_API BaseEdge<1, double>;
  template class G2O_CORE_API BaseEdge<1, Eigen::Matrix<double, 1, 1> >;
  template class G2O_CORE_API BaseEdge<2, Eigen::Matrix<double, 2, 1> >;
  template class G2O_CORE_API BaseEdge<3, Eigen::Matrix<double, 3, 1> >;
  template class G2O_CORE_API BaseEdge<4, Eigen::Matrix<double, 4, 1> >;
  template class G2O_CORE_API BaseEdge<5, Eigen::Matrix<double, 5, 1> >;
  template class G2O_CORE_API BaseEdge<6, Eigen::Matrix<double, 6, 1> >;
  template class G2O_CORE_API BaseEdge<7, Eigen::Matrix<double, 7, 1> >;
  template class G2O_CORE_API BaseEdge<6, Eigen::Isometry3d>;

G2O_END_NAMESPACE
