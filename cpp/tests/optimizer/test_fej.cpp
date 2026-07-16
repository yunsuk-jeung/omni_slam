#include <gtest/gtest.h>

#include "optimizer/cost_function.hpp"
#include "optimizer/parameterization.hpp"
#include "optimizer/relative_pose.hpp"

namespace omni_slam {
namespace {

struct BearingEval {
  Eigen::Vector2d                              r;
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_obs;
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_host;
};

BearingEval evaluate(BearingCost&           cost,
                     const Sophus::SE3d&    T_w_b_obs,
                     const Sophus::SE3d&    T_w_b_host,
                     const Eigen::Vector3d& bearing,
                     double                 inv_dist) {
  const Eigen::Vector6d pose_obs  = SE3BoxplusManifold::to_params(T_w_b_obs);
  const Eigen::Vector6d pose_host = SE3BoxplusManifold::to_params(T_w_b_host);
  const double*         params[4] = {pose_obs.data(),
                                     pose_host.data(),
                                     bearing.data(),
                                     &inv_dist};

  BearingEval out;
  double*     jacobians[4] = {out.J_obs.data(),
                              out.J_host.data(),
                              nullptr,
                              nullptr};
  EXPECT_TRUE(cost.Evaluate(params, out.r.data(), jacobians));
  return out;
}

class FejBearingCostTest : public ::testing::Test {
 protected:
  void SetUp() override {
    T_b_c_obs_  = Sophus::SE3d(Sophus::SO3d::exp({0.01, -0.02, 0.03}),
                              Eigen::Vector3d(0.1, 0.0, 0.02));
    T_b_c_host_ = Sophus::SE3d(Sophus::SO3d::exp({-0.02, 0.01, 0.0}),
                               Eigen::Vector3d(-0.1, 0.01, 0.0));

    T_host_ = Sophus::SE3d(Sophus::SO3d::exp({0.05, 0.1, -0.02}),
                           Eigen::Vector3d(0.4, -0.2, 1.1));
    T_obs_  = Sophus::SE3d(Sophus::SO3d::exp({-0.03, 0.08, 0.05}),
                          Eigen::Vector3d(0.9, 0.1, 1.0));

    bearing_  = Eigen::Vector3d(0.1, -0.05, 1.0).normalized();
    inv_dist_ = 0.25;

    const Eigen::Vector3d p_c_host = bearing_ / inv_dist_;
    const Eigen::Vector3d p_w      = (T_host_ * T_b_c_host_) * p_c_host;
    b_obs_ = ((T_obs_ * T_b_c_obs_).inverse() * p_w).normalized();

    // A distinctly different iterate, as if the optimizer moved the states
    // after the linearization point was frozen.
    T_host_moved_ = T_host_
                    * Sophus::SE3d(Sophus::SO3d::exp({0.02, -0.03, 0.01}),
                                   Eigen::Vector3d(0.05, 0.02, -0.04));
    T_obs_moved_ = T_obs_
                   * Sophus::SE3d(Sophus::SO3d::exp({-0.01, 0.02, 0.02}),
                                  Eigen::Vector3d(-0.03, 0.04, 0.02));
  }

  Sophus::SE3d    T_b_c_obs_, T_b_c_host_;
  Sophus::SE3d    T_host_, T_obs_;
  Sophus::SE3d    T_host_moved_, T_obs_moved_;
  Eigen::Vector3d bearing_, b_obs_;
  double          inv_dist_ = 0.0;
};

// With the linearization point equal to the current iterate, FEJ must be a
// no-op: identical residuals and jacobians to the default cost.
TEST_F(FejBearingCostTest, NoOpWhenLinEqualsCurrent) {
  BearingCost plain(b_obs_, T_b_c_obs_, T_b_c_host_, 1.0);
  BearingCost fej(b_obs_, T_b_c_obs_, T_b_c_host_, 1.0, T_obs_, T_host_);

  const BearingEval e0 = evaluate(plain, T_obs_, T_host_, bearing_, inv_dist_);
  const BearingEval e1 = evaluate(fej, T_obs_, T_host_, bearing_, inv_dist_);

  EXPECT_TRUE(e0.r.isApprox(e1.r, 1e-12));
  EXPECT_TRUE(e0.J_obs.isApprox(e1.J_obs, 1e-12));
  EXPECT_TRUE(e0.J_host.isApprox(e1.J_host, 1e-12));
}

// Once the iterate moves away from the frozen linearization point, the
// residual must follow the iterate while the chain-rule part of the pose
// jacobians stays pinned: FEJ and default jacobians must now differ.
TEST_F(FejBearingCostTest, JacobianPinnedResidualCurrent) {
  BearingCost plain(b_obs_, T_b_c_obs_, T_b_c_host_, 1.0);
  BearingCost fej(b_obs_, T_b_c_obs_, T_b_c_host_, 1.0, T_obs_, T_host_);

  const BearingEval plain_moved =
    evaluate(plain, T_obs_moved_, T_host_moved_, bearing_, inv_dist_);
  const BearingEval fej_moved =
    evaluate(fej, T_obs_moved_, T_host_moved_, bearing_, inv_dist_);

  // Residuals only see the current iterate — identical with and without FEJ.
  EXPECT_TRUE(plain_moved.r.isApprox(fej_moved.r, 1e-12));
  EXPECT_GT(plain_moved.r.norm(), 0.0);

  // Jacobians differ because the FEJ chain term is evaluated at the frozen
  // poses while the default one follows the iterate.
  EXPECT_FALSE(plain_moved.J_obs.isApprox(fej_moved.J_obs, 1e-9));
  EXPECT_FALSE(plain_moved.J_host.isApprox(fej_moved.J_host, 1e-9));
}

// compute_rel_pose jacobians must match numeric differentiation under the
// SE3BoxplusManifold increment convention (t += dt, R = R * Exp(dtheta)).
TEST_F(FejBearingCostTest, RelPoseJacobiansMatchNumeric) {
  Eigen::Matrix<double, 6, 6> d_rel_d_h, d_rel_d_t;
  const Sophus::SE3d          T_t_h = compute_rel_pose(T_host_,
                                              T_b_c_host_,
                                              T_obs_,
                                              T_b_c_obs_,
                                              &d_rel_d_h,
                                              &d_rel_d_t);

  const double eps = 1e-7;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector6d delta = Eigen::Vector6d::Zero();
    delta[i]              = eps;

    const auto boxplus = [&](const Sophus::SE3d& T) {
      return Sophus::SE3d(T.so3() * Sophus::SO3d::exp(delta.tail<3>()),
                          T.translation() + delta.head<3>());
    };

    // Perturb host.
    {
      const Sophus::SE3d T_t_h_pert =
        compute_rel_pose(boxplus(T_host_), T_b_c_host_, T_obs_, T_b_c_obs_);
      Eigen::Vector6d numeric;
      numeric.head<3>() = (T_t_h_pert.translation() - T_t_h.translation())
                          / eps;
      numeric.tail<3>() = (T_t_h.so3().inverse() * T_t_h_pert.so3()).log()
                          / eps;
      EXPECT_TRUE(numeric.isApprox(d_rel_d_h.col(i), 1e-5))
        << "host col " << i << "\nnumeric: " << numeric.transpose()
        << "\nanalytic: " << d_rel_d_h.col(i).transpose();
    }

    // Perturb target/obs.
    {
      const Sophus::SE3d T_t_h_pert =
        compute_rel_pose(T_host_, T_b_c_host_, boxplus(T_obs_), T_b_c_obs_);
      Eigen::Vector6d numeric;
      numeric.head<3>() = (T_t_h_pert.translation() - T_t_h.translation())
                          / eps;
      numeric.tail<3>() = (T_t_h.so3().inverse() * T_t_h_pert.so3()).log()
                          / eps;
      EXPECT_TRUE(numeric.isApprox(d_rel_d_t.col(i), 1e-5))
        << "obs col " << i << "\nnumeric: " << numeric.transpose()
        << "\nanalytic: " << d_rel_d_t.col(i).transpose();
    }
  }
}

}  // namespace
}  // namespace omni_slam
