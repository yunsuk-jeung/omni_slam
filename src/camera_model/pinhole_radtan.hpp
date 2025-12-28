#pragma once
#include <opencv2/opencv.hpp>
#include "camera_model/camera_model.hpp"

namespace omni_slam {
class PinholeRadialTangential : public CameraModelBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeRadialTangential() = default;
  explicit PinholeRadialTangential(const CameraParameter& params) {
    SetIntrinsics(params.intrinsics);
    SetDistortions(params.distortions);
  }
  ~PinholeRadialTangential() = default;

  Eigen::Vector2d distort(const Eigen::Vector2d& nuv) override {
    if (!has_distortion_) {
      return Eigen::Vector2d::Zero();
    }
    const double x  = nuv.x();
    const double y  = nuv.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double xy = x * y;
    const double r2 = x2 + y2;

    const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2;
    const double x_dist = x * radial + 2.0 * p1_ * xy + p2_ * (r2 + 2.0 * x2);
    const double y_dist = y * radial + p1_ * (r2 + 2.0 * y2) + 2.0 * p2_ * xy;

    return Eigen::Vector2d(x_dist - x, y_dist - y);
  }

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& undists) override {
    cv::undistortPoints(pts, undists, cv_K_, cv_D_);
  }

protected:
  void SetDistortions(const std::vector<double>& distortions) override {
    if (distortions.size() >= 4) {
      cv_D_ = (cv::Mat_<double>(1, 4) << distortions[0],
               distortions[1],
               distortions[2],
               distortions[3]);
      k1_ = distortions[0];
      k2_ = distortions[1];
      p1_ = distortions[2];
      p2_ = distortions[3];
      has_distortion_ = true;
    }
    else {
      has_distortion_ = false;
    }
  }

private:
  double k1_{0.0};
  double k2_{0.0};
  double p1_{0.0};
  double p2_{0.0};
};
}  // namespace omni_slam
