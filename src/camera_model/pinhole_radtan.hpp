#pragma once
#include <opencv2/opencv.hpp>
#include "camera_model/camera_model.hpp"

namespace omni_slam {
class PinholeRadialTangential : public CameraModelBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeRadialTangential()  = default;
  ~PinholeRadialTangential() = default;

  void distort(const Eigen::Vector2d& nuv, Eigen::Vector2d& dnuv) override {
    // double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    // mx2_u = nuv.x() * nuv.x();
    // my2_u = nuv.y() * nuv.y();
    // mxy_u = nuv.x() * nuv.y();

    // rho2_u = mx2_u + my2_u;

    // rad_dist_u = mD0 * rho2_u + mD1 * rho2_u * rho2_u;

    // dnuv << nuv.x() * rad_dist_u + 2.0 * mD2 * mxy_u + mD3 * (rho2_u + 2.0 * mx2_u),
    //   nuv.y() * rad_dist_u + 2.0 * mD3 * mxy_u + mD2 * (rho2_u + 2.0 * my2_u);
  }

  virtual void undistortPoints(std::vector<cv::Point2f>& pts,
                               std::vector<cv::Point2f>& undists) override {
    // cv::undistortPoints(pts, undists, mK, mD);
  }
};
}  // namespace omni_slam