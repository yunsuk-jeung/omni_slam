#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace omni_slam {

struct MargPosePrior {
  std::vector<uint64_t>     keyframe_ids;
  std::vector<Sophus::SE3d> T_w_b_lin;
  Eigen::MatrixXd           H;
  Eigen::VectorXd           b;
  std::vector<uint64_t>     keyframes_to_marg;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace omni_slam
