#include <array>
#include <vector>

#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "optimizer/marginalizer.hpp"

namespace omni_slam {
namespace {

MarginalizationPrior MakePrior(const std::vector<int>& block_sizes,
                               int                     residual_dim,
                               int                     state_dim) {
  MarginalizationPrior prior;
  prior.block_sizes_ = block_sizes;
  prior.J_.resize(residual_dim, state_dim);
  prior.r_.resize(residual_dim);
  prior.x0_.resize(state_dim);
  return prior;
}

}  // namespace

TEST(MarginalizationCostTest, ResidualAndJacobiansMatchExpectedLinearModel) {
  // residual = r + J * (x - x0)
  MarginalizationPrior prior = MakePrior({2, 3, 1}, 4, 6);

  prior.J_ << 1.0, 0.0, 2.0, -1.0, 0.5, 3.0, -2.0, 1.0, 0.0, 1.0, -0.5, 2.0,
    0.2, 0.3, -1.0, 0.0, 2.0, -0.7, 1.5, -0.2, 0.4, 0.8, -1.2, 0.1;
  prior.r_ << 0.1, -0.2, 0.3, -0.4;
  prior.x0_ << 1.0, 2.0, -1.0, 0.5, 0.3, -2.0;

  Eigen::Vector2d x0;
  Eigen::Vector3d x1;
  Eigen::VectorXd x2(1);
  x0 << 1.2, 1.7;
  x1 << -1.5, 1.0, 0.1;
  x2 << -1.0;

  MarginalizationCost          cost(prior);
  std::array<const double*, 3> params{x0.data(), x1.data(), x2.data()};

  Eigen::Vector4d                              residual;
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> J0;
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J1;
  Eigen::Matrix<double, 4, 1>                  J2;
  std::array<double*, 3> jacobians{J0.data(), J1.data(), J2.data()};

  ASSERT_TRUE(cost.Evaluate(params.data(), residual.data(), jacobians.data()));

  Eigen::Matrix<double, 6, 1> x;
  x << x0, x1, x2;
  const Eigen::Vector4d expected_residual = prior.r_
                                            + prior.J_ * (x - prior.x0_);
  EXPECT_LT((residual - expected_residual).norm(), 1e-12);

  EXPECT_LT((J0 - prior.J_.block<4, 2>(0, 0)).norm(), 1e-12);
  EXPECT_LT((J1 - prior.J_.block<4, 3>(0, 2)).norm(), 1e-12);
  EXPECT_LT((J2 - prior.J_.block<4, 1>(0, 5)).norm(), 1e-12);
}

TEST(MarginalizationCostTest, EvaluateWithoutJacobians) {
  MarginalizationPrior prior = MakePrior({2, 1}, 2, 3);
  prior.J_ << 2.0, -1.0, 0.5, 0.3, 1.2, -2.0;
  prior.r_ << 0.4, -0.7;
  prior.x0_ << 0.1, -0.2, 0.3;

  Eigen::Vector2d x0;
  Eigen::VectorXd x1(1);
  x0 << 0.2, -0.5;
  x1 << 0.8;

  MarginalizationCost          cost(prior);
  std::array<const double*, 2> params{x0.data(), x1.data()};

  Eigen::Vector2d residual;
  ASSERT_TRUE(cost.Evaluate(params.data(), residual.data(), nullptr));

  Eigen::Vector3d x;
  x << x0, x1;
  const Eigen::Vector2d expected_residual = prior.r_
                                            + prior.J_ * (x - prior.x0_);
  EXPECT_LT((residual - expected_residual).norm(), 1e-12);
}

TEST(MarginalizationCostTest, InvalidPriorDimensionsReturnFalse) {
  MarginalizationPrior prior;
  prior.block_sizes_ = {2, 2};
  prior.x0_          = Eigen::VectorXd::Zero(5);  // should be 4
  prior.r_           = Eigen::VectorXd::Zero(3);
  prior.J_           = Eigen::MatrixXd::Zero(3, 4);

  MarginalizationCost          cost(prior);
  Eigen::Vector2d              x0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d              x1 = Eigen::Vector2d::Zero();
  std::array<const double*, 2> params{x0.data(), x1.data()};
  Eigen::Vector3d              residual = Eigen::Vector3d::Zero();

  EXPECT_FALSE(cost.Evaluate(params.data(), residual.data(), nullptr));
}

TEST(MarginalizationCostTest,
     CeresOptimizationFindsLinearLeastSquaresSolution) {
  // Use full-rank square system: J = I, residual = r + (x - x0)
  // optimum is x* = x0 - r
  MarginalizationPrior prior = MakePrior({2, 1}, 3, 3);
  prior.J_.setIdentity();
  prior.r_ << 0.3, -0.5, 1.2;
  prior.x0_ << -1.0, 2.0, 0.4;

  Eigen::Vector2d x_block0(2.5, -3.0);
  double          x_block1 = 4.5;

  ceres::Problem problem;
  problem.AddParameterBlock(x_block0.data(), 2);
  problem.AddParameterBlock(&x_block1, 1);

  std::vector<double*> param_blocks{x_block0.data(), &x_block1};
  problem.AddResidualBlock(new MarginalizationCost(prior),
                           nullptr,
                           param_blocks);

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.max_num_iterations           = 20;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  EXPECT_TRUE(summary.IsSolutionUsable());

  Eigen::Vector3d x_est;
  x_est << x_block0[0], x_block0[1], x_block1;
  const Eigen::Vector3d x_star = prior.x0_ - prior.r_;
  EXPECT_LT((x_est - x_star).norm(), 1e-7);
}

}  // namespace omni_slam
