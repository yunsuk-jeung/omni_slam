#pragma once

#include <array>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "utils/types.hpp"

namespace omni_slam {

class CameraModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CameraModelBase()          = default;
  virtual ~CameraModelBase() = default;

  virtual void project(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv) = 0;

  virtual cv::Point2d project(const Eigen::Vector3d& xyz) = 0;

  virtual Eigen::Vector2d distort(const Eigen::Vector2d& n_uv) = 0;

  virtual void unproject(const std::vector<cv::Point2f> uvs,
                         std::vector<Eigen::Vector3d>&  bearings,
                         std::vector<bool>&             status) = 0;

  virtual bool unproject(const cv::Point2f& uv, Eigen::Vector3d& bearing_) = 0;

 protected:
  void intrinsics(const std::array<double, 4>& intrinsics) {
    cv_K_ = (cv::Mat_<double>(3, 3) << intrinsics[0],
             0.0,
             intrinsics[2],
             0.0,
             intrinsics[1],
             intrinsics[3],
             0.0,
             0.0,
             1.0);
    fx_   = intrinsics[0];
    fy_   = intrinsics[1];
    cx_   = intrinsics[2];
    cy_   = intrinsics[3];
  }

  virtual void distortions(const std::vector<double>& distortions) = 0;

  cv::Mat cv_K_, cv_D_;
  double  fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  bool    has_distortion_{false};
};

class CameraModelFactory {
 public:
  static std::unique_ptr<CameraModelBase> create(const CameraParameter& params);
};

}  // namespace omni_slam
