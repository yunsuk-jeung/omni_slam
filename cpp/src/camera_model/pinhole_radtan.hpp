#pragma once
#include <opencv2/opencv.hpp>

#include "camera_model/camera_model.hpp"

namespace omni_slam {
class PinholeRadialTangential : public CameraModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeRadialTangential() = default;
  explicit PinholeRadialTangential(const CameraParameter& params) {
    intrinsics(params.intrinsics);
    distortions(params.distortions);
  }
  ~PinholeRadialTangential() = default;

  void project(const Eigen::Vector3d& xyz, Eigen::Vector2d& uv) override {
    Eigen::Vector2d n_uv(xyz.x() / xyz.z(), xyz.y() / xyz.z());
    if (has_distortion_) {
      n_uv += distort(n_uv);
    }
    uv.x() = fx_ * n_uv.x() + cx_;
    uv.y() = fy_ * n_uv.y() + cy_;
  }

  cv::Point2d project(const Eigen::Vector3d& xyz) override {
    Eigen::Vector2d n_uv(xyz.x() / xyz.z(), xyz.y() / xyz.z());
    if (has_distortion_) {
      n_uv += distort(n_uv);
    }
    return cv::Point2d(fx_ * n_uv.x() + cx_, fy_ * n_uv.y() + cy_);
  }

  // Brown-Conrady (OpenCV) radial-tangential model: radial via k1,k2 plus
  // tangential via p1,p2.
  Eigen::Vector2d distort(const Eigen::Vector2d& n_uv) override {
    if (!has_distortion_) {
      return Eigen::Vector2d::Zero();
    }
    const double x  = n_uv.x();
    const double y  = n_uv.y();
    const double x2 = x * x;
    const double y2 = y * y;
    const double xy = x * y;
    const double r2 = x2 + y2;

    const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2;
    const double x_dist = x * radial + 2.0 * p1_ * xy + p2_ * (r2 + 2.0 * x2);
    const double y_dist = y * radial + p1_ * (r2 + 2.0 * y2) + 2.0 * p2_ * xy;

    return Eigen::Vector2d(x_dist - x, y_dist - y);
  }

  virtual void unproject(const std::vector<cv::Point2f> uvs,
                         std::vector<Eigen::Vector3d>&  bearings,
                         std::vector<bool>&             status) override {
    const size_t count = uvs.size();
    bearings.resize(count);
    status.assign(count, true);

    if (has_distortion_) {
      std::vector<cv::Point2f> norm_uvs;
      cv::undistortPoints(uvs, norm_uvs, cv_K_, cv_D_);
      for (size_t i = 0; i < count; ++i) {
        const auto& uv = norm_uvs[i];
        normalize_bearing(uv.x, uv.y, bearings[i]);
      }
      return;
    }

    for (size_t i = 0; i < count; ++i) {
      const auto&  uv = uvs[i];
      const double mx = (uv.x - cx_) / fx_;
      const double my = (uv.y - cy_) / fy_;
      normalize_bearing(mx, my, bearings[i]);
    }
  }

  virtual bool unproject(const cv::Point2f& uv,
                         Eigen::Vector3d&   bearing) override {
    double mx = (uv.x - cx_) / fx_;
    double my = (uv.y - cy_) / fy_;
    if (has_distortion_) {
      std::vector<cv::Point2f> src;
      std::vector<cv::Point2f> dst;
      src.reserve(1);
      dst.reserve(1);
      src.emplace_back(uv);
      cv::undistortPoints(src, dst, cv_K_, cv_D_);
      mx = dst.front().x;
      my = dst.front().y;
    }

    normalize_bearing(mx, my, bearing);
    return true;
  };

 protected:
  void distortions(const std::vector<double>& distortions) override {
    constexpr size_t kNumRadTanCoeffs = 4;  // k1, k2, p1, p2
    if (distortions.size() >= kNumRadTanCoeffs) {
      cv_D_           = (cv::Mat_<double>(1, 4) << distortions[0],
               distortions[1],
               distortions[2],
               distortions[3]);
      k1_             = distortions[0];
      k2_             = distortions[1];
      p1_             = distortions[2];
      p2_             = distortions[3];
      has_distortion_ = true;
    }
    else {
      has_distortion_ = false;
    }
  }

 private:
  static void normalize_bearing(double mx, double my, Eigen::Vector3d& out) {
    const double r2       = mx * mx + my * my;
    const double norm_inv = 1.0 / std::sqrt(1.0 + r2);
    out[0]                = mx * norm_inv;
    out[1]                = my * norm_inv;
    out[2]                = norm_inv;
  }

  double k1_{0.0};
  double k2_{0.0};
  double p1_{0.0};
  double p2_{0.0};
};
}  // namespace omni_slam
