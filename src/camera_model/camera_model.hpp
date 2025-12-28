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

  void project(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv) {
    Eigen::Vector2d n_uv(xyz.x() / xyz.z(), xyz.y() / xyz.z());
    if (has_distortion_) {
      n_uv += distort(n_uv);
    }
    uv.x() = fx_ * n_uv.x() + cx_;
    uv.y() = fy_ * n_uv.y() + cy_;
  }

  cv::Point2d project(const Eigen::Vector3d& xyz) {
    Eigen::Vector2d n_uv(xyz.x() / xyz.z(), xyz.y() / xyz.z());
    if (has_distortion_) {
      n_uv += distort(n_uv);
    }
    return cv::Point2d(fx_ * n_uv.x() + cx_, fy_ * n_uv.y() + cy_);
  }

  virtual Eigen::Vector2d distort(const Eigen::Vector2d& n_uv) = 0;

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& upts) = 0;

protected:
  void SetIntrinsics(const std::array<double, 4>& intrinsics) {
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

  virtual void SetDistortions(const std::vector<double>& distortions) = 0;

  cv::Mat cv_K_, cv_D_;
  double  fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  bool    has_distortion_{false};
};

class CameraModelFactory {
public:
  static std::unique_ptr<CameraModelBase> Create(const CameraParameter& params);
};

}  // namespace omni_slam
