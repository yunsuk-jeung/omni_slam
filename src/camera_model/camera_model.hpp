#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace omni_slam {

class CameraModelBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CameraModelBase()          = default;
  virtual ~CameraModelBase() = default;

  virtual void        project(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv)       = 0;
  virtual cv::Point2d project(const Eigen::Vector3d& xyz)                            = 0;
  virtual void        distort(const Eigen::Vector2d& input, Eigen::Vector2d& output) = 0;

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& upts) = 0;

  void SetIntrinsics(const std::vector<double>& intrinsics) {
    if (intrinsics.size() >= 4) {
      cv_K_ = (cv::Mat_<double>(3, 3) << intrinsics[0],
               0.0,
               intrinsics[2],
               0.0,
               intrinsics[1],
               intrinsics[3],
               0.0,
               0.0,
               1.0);
    }
  }

  void SetDistortions(const std::vector<double>& distortions) {
    if (distortions.size() >= 4) {
      cv_D_ = (cv::Mat_<double>(1, 4) << distortions[0],
               distortions[1],
               distortions[2],
               distortions[3]);
    }
  }

protected:
  cv::Mat cv_K_, cv_D_;
};
}  // namespace omni_slam
