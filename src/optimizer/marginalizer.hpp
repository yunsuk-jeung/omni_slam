#pragma once

#include <cstdint>
#include <set>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ceres/cost_function.h>

namespace omni_slam {
class MarginalizationCost : public ceres::CostFunction {
public:
  static constexpr int kPoseSize = 6;

  MarginalizationCost(const Eigen::MatrixXd& J,
                      const Eigen::VectorXd& r,
                      const Eigen::VectorXd& x0)
    : J_(J)
    , r_(r)
    , x0_(x0) {
    set_num_residuals(static_cast<int>(r_.size()));
    const int num_blocks = static_cast<int>(x0_.size()) / kPoseSize;
    mutable_parameter_block_sizes()->assign(num_blocks, kPoseSize);
  }

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    const int residual_dim = static_cast<int>(r_.size());
    const int num_blocks   = static_cast<int>(parameter_block_sizes().size());
    const int state_dim    = num_blocks * kPoseSize;

    if (state_dim != x0_.size() || J_.rows() != residual_dim || J_.cols() != state_dim) {
      return false;
    }

    Eigen::VectorXd dx(state_dim);
    for (int i = 0; i < num_blocks; ++i) {
      Eigen::Map<const Eigen::Matrix<double, kPoseSize, 1>> x_i(params[i]);
      dx.segment<kPoseSize>(i * kPoseSize) = x_i - x0_.segment<kPoseSize>(i * kPoseSize);
    }

    Eigen::Map<Eigen::VectorXd> residual_map(residuals, residual_dim);
    residual_map = r_ + J_ * dx;

    if (jacobians) {
      for (int i = 0; i < num_blocks; ++i) {
        if (!jacobians[i]) {
          continue;
        }
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, kPoseSize, Eigen::RowMajor>>
          J_i(jacobians[i], residual_dim, kPoseSize);
        J_i = J_.block(0, i * kPoseSize, residual_dim, kPoseSize);
      }
    }
    return true;
  }

private:
  Eigen::MatrixXd J_;
  Eigen::VectorXd r_;
  Eigen::VectorXd x0_;
};

class Marginalizer {
public:
  static constexpr int kPoseSize = 6;

  Marginalizer(uint64_t initial_frame_id, double initial_prior_weight) {
    frame_ids_.insert(initial_frame_id);
    x0_ = Eigen::VectorXd::Zero(kPoseSize);
    J_  = std::sqrt(initial_prior_weight)
         * Eigen::MatrixXd::Identity(kPoseSize, kPoseSize);
    r_ = Eigen::VectorXd::Zero(kPoseSize);
  }
  ~Marginalizer() = default;

  void Clear() {
    J_.resize(0, 0);
    r_.resize(0);
    x0_.resize(0);
    frame_ids_.clear();
  }

  void SetPrior(const std::set<uint64_t>& frame_ids,
                const Eigen::MatrixXd&    A,
                const Eigen::VectorXd&    b,
                const Eigen::VectorXd&    x0) {
    frame_ids_ = frame_ids;
    x0_        = x0;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(A);

    constexpr double eps = 1e-8;

    const Eigen::VectorXd eig_vals = saes.eigenvalues();
    const Eigen::MatrixXd eig_vecs = saes.eigenvectors();

    Eigen::VectorXd sqrt_vals(eig_vals.size());
    Eigen::VectorXd inv_sqrt_vals(eig_vals.size());
    for (int i = 0; i < eig_vals.size(); ++i) {
      if (eig_vals[i] > eps) {
        sqrt_vals[i]     = std::sqrt(eig_vals[i]);
        inv_sqrt_vals[i] = 1.0 / sqrt_vals[i];
      }
      else {
        sqrt_vals[i]     = 0.0;
        inv_sqrt_vals[i] = 0.0;
      }
    }

    J_ = sqrt_vals.asDiagonal() * eig_vecs.transpose();
    r_ = inv_sqrt_vals.asDiagonal() * eig_vecs.transpose() * b;
  }

  MarginalizationCost* CreateCost() const { return new MarginalizationCost(J_, r_, x0_); }

  const std::set<uint64_t>& GetFrameIds() const { return frame_ids_; }
  std::set<uint64_t>&       GetFrameIds() { return frame_ids_; }

private:
  Eigen::MatrixXd J_;
  Eigen::VectorXd r_;
  Eigen::VectorXd x0_;

  std::set<uint64_t> frame_ids_;
  std::set<uint64_t> preintegration_ids_;
};
}  // namespace omni_slam
