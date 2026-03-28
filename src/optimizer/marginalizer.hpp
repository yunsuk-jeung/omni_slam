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
  MarginalizationCost(const Eigen::MatrixXd&  J,
                      const Eigen::VectorXd&  r,
                      const Eigen::VectorXd&  x0,
                      const std::vector<int>& block_sizes)
    : J_(J)
    , r_(r)
    , x0_(x0)
    , block_sizes_(block_sizes) {
    set_num_residuals(static_cast<int>(r_.size()));
    mutable_parameter_block_sizes()->assign(block_sizes_.begin(), block_sizes_.end());
  }

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    const int residual_dim = static_cast<int>(r_.size());
    const int num_blocks   = static_cast<int>(block_sizes_.size());
    int       state_dim    = 0;
    for (const int block_size : block_sizes_) {
      state_dim += block_size;
    }

    if (state_dim != x0_.size() || J_.rows() != residual_dim || J_.cols() != state_dim) {
      return false;
    }

    Eigen::VectorXd dx(state_dim);
    int             col_offset = 0;
    for (int i = 0; i < num_blocks; ++i) {
      const int                         block_size = block_sizes_[i];
      Eigen::Map<const Eigen::VectorXd> x_i(params[i], block_size);
      dx.segment(col_offset, block_size) = x_i - x0_.segment(col_offset, block_size);
      col_offset += block_size;
    }

    Eigen::Map<Eigen::VectorXd> residual_map(residuals, residual_dim);
    residual_map = r_ + J_ * dx;

    if (jacobians) {
      col_offset = 0;
      for (int i = 0; i < num_blocks; ++i) {
        if (!jacobians[i]) {
          col_offset += block_sizes_[i];
          continue;
        }
        const int block_size = block_sizes_[i];
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
          J_i(jacobians[i], residual_dim, block_size);
        J_i = J_.block(0, col_offset, residual_dim, block_size);
        col_offset += block_size;
      }
    }
    return true;
  }

private:
  Eigen::MatrixXd  J_;
  Eigen::VectorXd  r_;
  Eigen::VectorXd  x0_;
  std::vector<int> block_sizes_;
};

class Marginalizer {
public:
  static constexpr int kPoseSize = 6;

  Marginalizer(uint64_t initial_frame_id,
               double   initial_prior_weight,
               double   initial_bias_weight = 0.0) {
    constexpr int kBiasSize = 3;
    if (!std::isfinite(initial_prior_weight) || initial_prior_weight <= 0.0) {
      initial_prior_weight = 1e10;
    }
    frame_ids_.insert(initial_frame_id);

    const bool use_bias_blocks = std::isfinite(initial_bias_weight)
                                 && initial_bias_weight > 0.0;
    if (use_bias_blocks) {
      preintegration_ids_.insert(initial_frame_id);
      x0_ = Eigen::VectorXd::Zero(kPoseSize + 2 * kBiasSize);
      J_  = Eigen::MatrixXd::Zero(x0_.size(), x0_.size());
      J_.topLeftCorner(kPoseSize, kPoseSize) = std::sqrt(initial_prior_weight)
                                               * Eigen::MatrixXd::Identity(kPoseSize,
                                                                           kPoseSize);
      J_.block(kPoseSize,
               kPoseSize,
               kBiasSize,
               kBiasSize) = std::sqrt(initial_bias_weight)
                            * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);
      J_.block(kPoseSize + kBiasSize,
               kPoseSize + kBiasSize,
               kBiasSize,
               kBiasSize) = std::sqrt(initial_bias_weight)
                            * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);
      block_sizes_ = {kPoseSize, kBiasSize, kBiasSize};
    }
    else {
      x0_ = Eigen::VectorXd::Zero(kPoseSize);
      J_  = std::sqrt(initial_prior_weight)
           * Eigen::MatrixXd::Identity(kPoseSize, kPoseSize);
      block_sizes_ = {kPoseSize};
    }
    r_ = Eigen::VectorXd::Zero(x0_.size());
  }
  ~Marginalizer() = default;

  void Clear() {
    J_.resize(0, 0);
    r_.resize(0);
    x0_.resize(0);
    frame_ids_.clear();
    preintegration_ids_.clear();
    block_sizes_.clear();
  }

  void SetPrior(const std::set<uint64_t>& frame_ids,
                const Eigen::MatrixXd&    A,
                const Eigen::VectorXd&    b,
                const Eigen::VectorXd&    x0) {
    std::vector<int> block_sizes(frame_ids.size(), kPoseSize);
    SetPrior(frame_ids, {}, block_sizes, A, b, x0);
  }

  void SetPrior(const std::set<uint64_t>& frame_ids,
                const std::set<uint64_t>& preintegration_ids,
                const std::vector<int>&   block_sizes,
                const Eigen::MatrixXd&    A,
                const Eigen::VectorXd&    b,
                const Eigen::VectorXd&    x0) {
    frame_ids_          = frame_ids;
    preintegration_ids_ = preintegration_ids;
    block_sizes_        = block_sizes;
    x0_                 = x0;

    if (x0_.size() == 0) {
      J_.resize(0, 0);
      r_.resize(0);
      return;
    }

    int total_block_size = 0;
    for (const int block_size : block_sizes_) {
      total_block_size += block_size;
    }
    if (total_block_size != x0_.size()) {
      J_.resize(0, 0);
      r_.resize(0);
      x0_.resize(0);
      frame_ids_.clear();
      preintegration_ids_.clear();
      block_sizes_.clear();
      return;
    }

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

  MarginalizationCost* CreateCost() const {
    return new MarginalizationCost(J_, r_, x0_, block_sizes_);
  }

  const std::set<uint64_t>& GetFrameIds() const { return frame_ids_; }
  std::set<uint64_t>&       GetFrameIds() { return frame_ids_; }
  const std::set<uint64_t>& GetPreintegrationIds() const { return preintegration_ids_; }
  std::set<uint64_t>&       GetPreintegrationIds() { return preintegration_ids_; }
  const std::vector<int>&   GetBlockSizes() const { return block_sizes_; }

private:
  Eigen::MatrixXd J_;
  Eigen::VectorXd r_;
  Eigen::VectorXd x0_;

  std::set<uint64_t> frame_ids_;
  std::set<uint64_t> preintegration_ids_;
  std::vector<int>   block_sizes_;
};
}  // namespace omni_slam
