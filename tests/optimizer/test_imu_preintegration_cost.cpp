#include <gtest/gtest.h>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include "odometry/imu_preintegration.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/parameterization.hpp"

namespace omni_slam {

class ImuPreintegrationCostTest : public ::testing::Test {
protected:
  static constexpr double kEps = 1e-6;

  // Build a preintegration from constant IMU measurements
  ImuPreintegration MakePreintegration(
    const Eigen::Vector3d& acc,
    const Eigen::Vector3d& gyr,
    double                 dt_total,
    int                    num_steps,
    const Eigen::Vector3d& bias_acc = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& bias_gyr = Eigen::Vector3d::Zero()) {
    ImuPreintegration preint(/*from_frame_id=*/0,
                             /*to_frame_id=*/1,
                             bias_acc,
                             bias_gyr);

    const double dt_step = dt_total / num_steps;
    for (int i = 0; i <= num_steps; ++i) {
      ImuData imu;
      imu.t_ns = static_cast<int64_t>(i * dt_step * 1e9);
      imu.acc  = acc;
      imu.gyr  = gyr;
      if (i > 0) {
        ImuData imu_prev;
        imu_prev.t_ns = static_cast<int64_t>((i - 1) * dt_step * 1e9);
        imu_prev.acc  = acc;
        imu_prev.gyr  = gyr;
        preint.IntegrateMeasurement(imu_prev, imu);
      }
    }
    return preint;
  }

  // Evaluate residual for given state
  Eigen::Matrix<double, 15, 1> EvaluateResidual(const ImuPreintegration& preint,
                                                const Eigen::Vector3d&   gravity,
                                                const Sophus::SE3d&      T_w_b_i,
                                                const Sophus::SE3d&      T_w_b_j,
                                                const Eigen::Vector3d&   v_i,
                                                const Eigen::Vector3d&   v_j,
                                                const Eigen::Vector3d&   ba_i,
                                                const Eigen::Vector3d&   bg_i,
                                                const Eigen::Vector3d&   ba_j,
                                                const Eigen::Vector3d&   bg_j) {
    ImuPreintegrationCost cost(preint, gravity);

    Eigen::Vector6d pose_i = SE3BoxplusManifold::ToParams(T_w_b_i);
    Eigen::Vector6d pose_j = SE3BoxplusManifold::ToParams(T_w_b_j);

    Eigen::Vector3d vi  = v_i;
    Eigen::Vector3d vj  = v_j;
    Eigen::Vector3d bai = ba_i;
    Eigen::Vector3d bgi = bg_i;
    Eigen::Vector3d baj = ba_j;
    Eigen::Vector3d bgj = bg_j;

    const double* params[] = {pose_i.data(),
                              pose_j.data(),
                              vi.data(),
                              vj.data(),
                              bai.data(),
                              bgi.data(),
                              baj.data(),
                              bgj.data()};

    Eigen::Matrix<double, 15, 1> residual;
    cost.Evaluate(params, residual.data(), nullptr);
    return residual;
  }
};

// Zero-motion: body at rest, gravity measured by accelerometer.
// If the states are perfectly consistent, residual should be ~zero.
TEST_F(ImuPreintegrationCostTest, ZeroMotionResidual) {
  const Eigen::Vector3d gravity(0, 0, -9.81);
  const Eigen::Vector3d acc_meas = -gravity;  // accelerometer reads -g when static
  const Eigen::Vector3d gyr_meas = Eigen::Vector3d::Zero();

  const double dt     = 0.1;
  const int    steps  = 10;
  auto         preint = MakePreintegration(acc_meas, gyr_meas, dt, steps);

  const Sophus::SE3d    T_w_b = Sophus::SE3d();  // identity
  const Eigen::Vector3d v     = Eigen::Vector3d::Zero();
  const Eigen::Vector3d ba    = Eigen::Vector3d::Zero();
  const Eigen::Vector3d bg    = Eigen::Vector3d::Zero();

  auto residual = EvaluateResidual(preint,
                                   gravity,
                                   T_w_b,
                                   T_w_b,  // same pose
                                   v,
                                   v,  // same velocity
                                   ba,
                                   bg,
                                   ba,
                                   bg);

  EXPECT_NEAR(residual.norm(), 0.0, 1e-6)
    << "Residual should be zero for stationary body\n"
    << "residual: " << residual.transpose();
}

// Constant velocity with gravity: body falls freely.
// p_j = p_i + v_i * dt + 0.5 * g * dt^2, v_j = v_i + g * dt
TEST_F(ImuPreintegrationCostTest, FreeFallResidual) {
  const Eigen::Vector3d gravity(0, 0, -9.81);
  // In free fall, accelerometer reads zero
  const Eigen::Vector3d acc_meas = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gyr_meas = Eigen::Vector3d::Zero();

  const double dt     = 0.5;
  const int    steps  = 50;
  auto         preint = MakePreintegration(acc_meas, gyr_meas, dt, steps);

  const Eigen::Vector3d p_i(0, 0, 0);
  const Eigen::Vector3d v_i(1, 0, 0);
  const Eigen::Vector3d p_j = p_i + v_i * dt + 0.5 * gravity * dt * dt;
  const Eigen::Vector3d v_j = v_i + gravity * dt;

  const Sophus::SE3d    T_i(Sophus::SO3d(), p_i);
  const Sophus::SE3d    T_j(Sophus::SO3d(), p_j);
  const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  const Eigen::Vector3d bg = Eigen::Vector3d::Zero();

  auto residual = EvaluateResidual(preint, gravity, T_i, T_j, v_i, v_j, ba, bg, ba, bg);

  EXPECT_NEAR(residual.norm(), 0.0, 1e-6)
    << "Residual should be zero for consistent free-fall state\n"
    << "residual: " << residual.transpose();
}

// Pure rotation: body rotates in place with zero gravity and no linear acceleration.
TEST_F(ImuPreintegrationCostTest, PureRotationResidual) {
  const Eigen::Vector3d gravity  = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc_meas = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gyr_meas(0.1, 0, 0);  // rotate around x-axis

  const double dt     = 0.5;
  const int    steps  = 50;
  auto         preint = MakePreintegration(acc_meas, gyr_meas, dt, steps);

  const Sophus::SO3d R_i;
  const Sophus::SO3d R_j = R_i * preint.GetDeltaR();

  const Sophus::SE3d    T_i(R_i, Eigen::Vector3d::Zero());
  const Sophus::SE3d    T_j(R_j, Eigen::Vector3d::Zero());
  const Eigen::Vector3d v  = Eigen::Vector3d::Zero();
  const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  const Eigen::Vector3d bg = Eigen::Vector3d::Zero();

  auto residual = EvaluateResidual(preint, gravity, T_i, T_j, v, v, ba, bg, ba, bg);

  EXPECT_NEAR(residual.norm(), 0.0, 1e-6)
    << "Residual should be zero for consistent pure rotation\n"
    << "residual: " << residual.transpose();
}

// Jacobian check: compare analytic Jacobian against numeric (finite diff).
TEST_F(ImuPreintegrationCostTest, JacobianCheck) {
  const Eigen::Vector3d gravity(0, 0, -9.81);
  const Eigen::Vector3d acc_meas(0.5, -0.3, 9.81);
  const Eigen::Vector3d gyr_meas(0.1, -0.05, 0.02);

  auto preint = MakePreintegration(acc_meas, gyr_meas, 0.2, 20);

  ceres::CostFunction* cost = new ImuPreintegrationCost(preint, gravity);

  // Non-trivial state
  Eigen::Vector6d pose_i, pose_j;
  pose_i << 1.0, 0.5, -0.3, 0.01, -0.02, 0.03;
  pose_j << 1.2, 0.6, -0.1, 0.02, -0.01, 0.01;

  Eigen::Vector3d v_i(0.5, -0.1, 0.2);
  Eigen::Vector3d v_j(0.6, -0.05, 0.15);
  Eigen::Vector3d ba_i(0.01, -0.02, 0.005);
  Eigen::Vector3d bg_i(0.001, -0.001, 0.0005);
  Eigen::Vector3d ba_j(0.012, -0.018, 0.006);
  Eigen::Vector3d bg_j(0.0012, -0.0008, 0.0004);

  std::vector<const double*> params = {pose_i.data(),
                                       pose_j.data(),
                                       v_i.data(),
                                       v_j.data(),
                                       ba_i.data(),
                                       bg_i.data(),
                                       ba_j.data(),
                                       bg_j.data()};

  // Use Ceres' numeric diff checker
  ceres::NumericDiffOptions numeric_diff_options;
  numeric_diff_options.relative_step_size = 1e-7;

  std::vector<std::vector<double>>     jacobian_diff;
  ceres::GradientChecker               checker(cost, nullptr, numeric_diff_options);
  ceres::GradientChecker::ProbeResults results;
  bool                                 ok = checker.Probe(params.data(), 1e-4, &results);

  EXPECT_TRUE(ok) << "Analytic/numeric Jacobian mismatch.\n"
                  << "Max relative error: " << results.maximum_relative_error << "\n"
                  << results.error_log;

  delete cost;
}

// Bias residual: changing bias between i and j produces non-zero residual.
// Note: sqrt_information_ is a full 15x15 matrix, so bias changes propagate
// to all residual components after weighting.
TEST_F(ImuPreintegrationCostTest, BiasChangeResidual) {
  const Eigen::Vector3d gravity(0, 0, -9.81);
  const Eigen::Vector3d acc_meas = -gravity;
  const Eigen::Vector3d gyr_meas = Eigen::Vector3d::Zero();

  auto preint = MakePreintegration(acc_meas, gyr_meas, 0.1, 10);

  const Sophus::SE3d    T_w_b;
  const Eigen::Vector3d v  = Eigen::Vector3d::Zero();
  const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  const Eigen::Vector3d bg = Eigen::Vector3d::Zero();

  // Same bias → zero residual
  auto
    residual_same = EvaluateResidual(preint, gravity, T_w_b, T_w_b, v, v, ba, bg, ba, bg);
  EXPECT_NEAR(residual_same.norm(), 0.0, 1e-6);

  // Different bias at j → non-zero residual
  const Eigen::Vector3d ba_j(0.1, 0.0, 0.0);
  const Eigen::Vector3d bg_j(0.0, 0.01, 0.0);

  auto residual_diff = EvaluateResidual(preint,
                                        gravity,
                                        T_w_b,
                                        T_w_b,
                                        v,
                                        v,
                                        ba,
                                        bg,
                                        ba_j,
                                        bg_j);

  EXPECT_GT(residual_diff.norm(), 0.01)
    << "Residual should be non-zero for mismatched bias";
}

// Zero dt: should return zero residual.
TEST_F(ImuPreintegrationCostTest, ZeroDtReturnsZero) {
  ImuPreintegration preint(0, 1);
  // No measurements integrated → dt = 0

  ImuPreintegrationCost cost(preint, Eigen::Vector3d(0, 0, -9.81));

  Eigen::Vector6d pose = Eigen::Vector6d::Zero();
  Eigen::Vector3d v    = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba   = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg   = Eigen::Vector3d::Zero();

  const double* params[] = {pose.data(),
                            pose.data(),
                            v.data(),
                            v.data(),
                            ba.data(),
                            bg.data(),
                            ba.data(),
                            bg.data()};

  Eigen::Matrix<double, 15, 1> residual;
  bool                         ok = cost.Evaluate(params, residual.data(), nullptr);

  EXPECT_TRUE(ok);
  EXPECT_NEAR(residual.norm(), 0.0, 1e-12);
}

TEST_F(ImuPreintegrationCostTest, CovarianceDoesNotDependOnStepSubdivision) {
  auto integrate = [](int steps) {
    ImuPreintegration::Parameters params;
    params.acc_noise_sigma   = 0.08;
    params.gyr_noise_sigma   = 0.004;
    params.acc_bias_rw_sigma = 0.00004;
    params.gyr_bias_rw_sigma = 0.000002;

    ImuPreintegration preint(0,
                             1,
                             Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero(),
                             params);

    const double dt_step = 1.0 / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
      ImuData imu0;
      imu0.t_ns = static_cast<int64_t>(i * dt_step * 1e9);
      imu0.acc.setZero();
      imu0.gyr.setZero();

      ImuData imu1;
      imu1.t_ns = static_cast<int64_t>((i + 1) * dt_step * 1e9);
      imu1.acc.setZero();
      imu1.gyr.setZero();
      preint.IntegrateMeasurement(imu0, imu1);
    }
    return preint.GetCovariance();
  };

  const Eigen::Matrix15d cov_10  = integrate(10);
  const Eigen::Matrix15d cov_100 = integrate(100);

  const double vel_trace_10  = cov_10.block<3, 3>(6, 6).trace();
  const double vel_trace_100 = cov_100.block<3, 3>(6, 6).trace();
  const double ba_trace_10   = cov_10.block<3, 3>(9, 9).trace();
  const double ba_trace_100  = cov_100.block<3, 3>(9, 9).trace();

  EXPECT_NEAR(vel_trace_10, vel_trace_100, vel_trace_10 * 1e-6);
  EXPECT_NEAR(ba_trace_10, ba_trace_100, ba_trace_10 * 1e-6);
}

// Ceres optimization: given wrong initial velocity, optimizer should recover.
TEST_F(ImuPreintegrationCostTest, CeresOptimizationRecoversVelocity) {
  const Eigen::Vector3d gravity(0, 0, -9.81);
  const Eigen::Vector3d acc_meas = -gravity;
  const Eigen::Vector3d gyr_meas = Eigen::Vector3d::Zero();

  const double dt     = 1.0;
  auto         preint = MakePreintegration(acc_meas, gyr_meas, dt, 100);

  // Ground truth: stationary
  Eigen::Vector6d pose_i = Eigen::Vector6d::Zero();
  Eigen::Vector6d pose_j = Eigen::Vector6d::Zero();
  Eigen::Vector3d v_i(0, 0, 0);
  Eigen::Vector3d v_j(0, 0, 0);
  Eigen::Vector3d ba_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_j = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_j = Eigen::Vector3d::Zero();

  // Perturb velocity
  v_i = Eigen::Vector3d(1.0, -0.5, 0.3);
  v_j = Eigen::Vector3d(-0.5, 1.0, -0.2);

  ceres::Problem problem;
  auto*          manifold = new SE3BoxplusManifold();
  problem.AddParameterBlock(pose_i.data(), 6, manifold);
  problem.AddParameterBlock(pose_j.data(), 6, manifold);

  ceres::CostFunction* cost = new ImuPreintegrationCost(preint, gravity);
  problem.AddResidualBlock(cost,
                           nullptr,
                           pose_i.data(),
                           pose_j.data(),
                           v_i.data(),
                           v_j.data(),
                           ba_i.data(),
                           bg_i.data(),
                           ba_j.data(),
                           bg_j.data());

  problem.SetParameterBlockConstant(pose_i.data());
  problem.SetParameterBlockConstant(pose_j.data());
  problem.SetParameterBlockConstant(ba_i.data());
  problem.SetParameterBlockConstant(bg_i.data());
  problem.SetParameterBlockConstant(ba_j.data());
  problem.SetParameterBlockConstant(bg_j.data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 50;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  EXPECT_NEAR(v_i.norm(), 0.0, 1e-4)
    << "v_i should converge to zero, got: " << v_i.transpose();
  EXPECT_NEAR(v_j.norm(), 0.0, 1e-4)
    << "v_j should converge to zero, got: " << v_j.transpose();
}

}  // namespace omni_slam
