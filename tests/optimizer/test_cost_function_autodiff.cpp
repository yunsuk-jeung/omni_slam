#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include <ceres/ceres.h>

#include "odometry/imu_preintegration.hpp"
#include "optimizer/cost_function.hpp"

namespace omni_slam {
namespace {

template <size_t N>
using DoublePtrArray = std::array<double*, N>;

template <size_t N>
using ConstDoublePtrArray = std::array<const double*, N>;

template <size_t N>
using MatrixArray = std::array<Eigen::MatrixXd, N>;

template <size_t N>
void EvaluateCostFunction(ceres::CostFunction*      cost,
                          const ConstDoublePtrArray<N>& params,
                          const std::array<int, N>&  block_sizes,
                          Eigen::VectorXd*           residual,
                          MatrixArray<N>*            jacobians) {
  ASSERT_NE(cost, nullptr);
  ASSERT_NE(residual, nullptr);
  ASSERT_NE(jacobians, nullptr);
  ASSERT_EQ(cost->parameter_block_sizes().size(), N);

  residual->resize(cost->num_residuals());
  DoublePtrArray<N> jacobian_ptrs{};
  for (size_t i = 0; i < N; ++i) {
    (*jacobians)[i].resize(cost->num_residuals(), block_sizes[i]);
    jacobian_ptrs[i] = (*jacobians)[i].data();
  }

  bool ok = cost->Evaluate(params.data(), residual->data(), jacobian_ptrs.data());
  ASSERT_TRUE(ok);
}

void ExpectResidualAndJacobiansClose(const Eigen::VectorXd&  residual_manual,
                                     const Eigen::VectorXd&  residual_auto,
                                     const MatrixArray<1>&   jacobian_manual,
                                     const MatrixArray<1>&   jacobian_auto,
                                     double                  residual_tol,
                                     double                  jacobian_tol) {
  ASSERT_EQ(residual_manual.size(), residual_auto.size());
  EXPECT_LT((residual_manual - residual_auto).norm(), residual_tol);

  ASSERT_EQ(jacobian_manual[0].rows(), jacobian_auto[0].rows());
  ASSERT_EQ(jacobian_manual[0].cols(), jacobian_auto[0].cols());
  const double denom = std::max(1.0, jacobian_auto[0].norm());
  EXPECT_LT((jacobian_manual[0] - jacobian_auto[0]).norm() / denom, jacobian_tol);
}

ImuPreintegration MakePreintegration(const Eigen::Vector3d& acc,
                                     const Eigen::Vector3d& gyr,
                                     double                 dt_total,
                                     int                    num_steps) {
  ImuPreintegration preint(/*from_frame_id=*/0,
                           /*to_frame_id=*/1,
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero());

  const double dt_step = dt_total / static_cast<double>(num_steps);
  for (int i = 0; i < num_steps; ++i) {
    ImuData imu0;
    imu0.t_ns = static_cast<int64_t>(i * dt_step * 1e9);
    imu0.acc  = acc;
    imu0.gyr  = gyr;

    ImuData imu1;
    imu1.t_ns = static_cast<int64_t>((i + 1) * dt_step * 1e9);
    imu1.acc  = acc;
    imu1.gyr  = gyr;
    preint.IntegrateMeasurement(imu0, imu1);
  }
  return preint;
}

}  // namespace

TEST(CostFunctionAutoDiffCompare, PoseOnlyBearingCost) {
  const Eigen::Vector3d p_w(0.4, -0.2, 3.0);
  const Eigen::Vector3d b_obs = Eigen::Vector3d(0.1, -0.05, 1.0).normalized();
  const Sophus::SE3d    T_b_c(Sophus::SO3d::exp(Eigen::Vector3d(0.01, -0.02, 0.03)),
                           Eigen::Vector3d(0.02, -0.01, 0.03));
  constexpr double      kScale = 20.0;

  Eigen::Vector6d pose;
  pose << 0.1, 0.02, 0.2, 0.02, -0.01, 0.03;

  PoseOnlyBearingCost manual_cost(p_w, b_obs, T_b_c, kScale);
  auto* auto_functor = new PoseOnlyBearingCostAuto(p_w, b_obs, T_b_c, kScale);
  std::unique_ptr<ceres::CostFunction> auto_cost(
    new ceres::AutoDiffCostFunction<PoseOnlyBearingCostAuto, 2, 6>(auto_functor));

  ConstDoublePtrArray<1> params{pose.data()};
  std::array<int, 1>     block_sizes{6};

  Eigen::VectorXd residual_manual;
  Eigen::VectorXd residual_auto;
  MatrixArray<1>  jacobian_manual;
  MatrixArray<1>  jacobian_auto;

  EvaluateCostFunction<1>(&manual_cost, params, block_sizes, &residual_manual, &jacobian_manual);
  EvaluateCostFunction<1>(auto_cost.get(), params, block_sizes, &residual_auto, &jacobian_auto);

  ExpectResidualAndJacobiansClose(residual_manual,
                                  residual_auto,
                                  jacobian_manual,
                                  jacobian_auto,
                                  1e-9,
                                  1e-6);
}

TEST(CostFunctionAutoDiffCompare, BearingStereoCost) {
  const Eigen::Vector3d b_obs = Eigen::Vector3d(0.02, -0.03, 1.0).normalized();
  const Sophus::SE3d    T_b_c_obs(Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.0)),
                               Eigen::Vector3d(0.0, 0.0, 0.0));
  const Sophus::SE3d    T_b_c_host(Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.001)),
                                Eigen::Vector3d(0.11, 0.0, 0.0));
  constexpr double      kScale = 20.0;

  Eigen::Vector3d bearing = Eigen::Vector3d(0.01, -0.02, 1.0).normalized();
  double          inv_dist = 0.4;

  BearingStereoCost manual_cost(b_obs, T_b_c_obs, T_b_c_host, kScale);
  auto* auto_functor = new BearingStereoCostAuto(b_obs, T_b_c_obs, T_b_c_host, kScale);
  std::unique_ptr<ceres::CostFunction> auto_cost(
    new ceres::AutoDiffCostFunction<BearingStereoCostAuto, 2, 3, 1>(auto_functor));

  ConstDoublePtrArray<2> params{bearing.data(), &inv_dist};
  std::array<int, 2>     block_sizes{3, 1};

  Eigen::VectorXd residual_manual;
  Eigen::VectorXd residual_auto;
  MatrixArray<2>  jacobian_manual;
  MatrixArray<2>  jacobian_auto;

  EvaluateCostFunction<2>(&manual_cost, params, block_sizes, &residual_manual, &jacobian_manual);
  EvaluateCostFunction<2>(auto_cost.get(), params, block_sizes, &residual_auto, &jacobian_auto);

  EXPECT_LT((residual_manual - residual_auto).norm(), 1e-9);

  for (size_t i = 0; i < 2; ++i) {
    const double denom = std::max(1.0, jacobian_auto[i].norm());
    EXPECT_LT((jacobian_manual[i] - jacobian_auto[i]).norm() / denom, 1e-6);
  }
}

TEST(CostFunctionAutoDiffCompare, ImuPreintegrationCost) {
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d acc_meas(0.2, -0.1, 9.75);
  const Eigen::Vector3d gyr_meas(0.02, -0.01, 0.03);
  ImuPreintegration      preint = MakePreintegration(acc_meas, gyr_meas, 0.2, 20);

  Eigen::Vector6d pose_i;
  Eigen::Vector6d pose_j;
  pose_i << 0.1, 0.0, 0.2, 0.01, -0.02, 0.03;
  pose_j << 0.2, 0.05, 0.25, 0.015, -0.018, 0.028;

  Eigen::Vector3d v_i(0.3, -0.2, 0.1);
  Eigen::Vector3d v_j(0.35, -0.18, 0.08);
  Eigen::Vector3d ba_i(0.01, -0.005, 0.002);
  Eigen::Vector3d bg_i(0.001, -0.002, 0.0007);
  Eigen::Vector3d ba_j(0.012, -0.004, 0.0025);
  Eigen::Vector3d bg_j(0.0015, -0.0018, 0.0006);

  ImuPreintegrationCost manual_cost(preint, gravity);
  std::unique_ptr<ceres::CostFunction> auto_cost(
    new ImuPreintegrationAutoDiffCost(new ImuPreintegrationCostAuto(preint, gravity)));

  ConstDoublePtrArray<8> params{pose_i.data(),
                                pose_j.data(),
                                v_i.data(),
                                v_j.data(),
                                ba_i.data(),
                                bg_i.data(),
                                ba_j.data(),
                                bg_j.data()};
  std::array<int, 8>     block_sizes{6, 6, 3, 3, 3, 3, 3, 3};

  Eigen::VectorXd residual_manual;
  Eigen::VectorXd residual_auto;
  MatrixArray<8>  jacobian_manual;
  MatrixArray<8>  jacobian_auto;

  EvaluateCostFunction<8>(&manual_cost, params, block_sizes, &residual_manual, &jacobian_manual);
  EvaluateCostFunction<8>(auto_cost.get(), params, block_sizes, &residual_auto, &jacobian_auto);

  EXPECT_LT((residual_manual - residual_auto).norm(), 1e-9);
  for (size_t i = 0; i < 8; ++i) {
    const double denom = std::max(1.0, jacobian_auto[i].norm());
    EXPECT_LT((jacobian_manual[i] - jacobian_auto[i]).norm() / denom, 1e-4)
      << "Jacobian block mismatch at index " << i;
  }
}

}  // namespace omni_slam
