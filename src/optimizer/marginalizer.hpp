#pragma once

#include <cstdint>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <ceres/cost_function.h>

namespace omni_slam {

struct MarginalizationPrior {
  Eigen::MatrixXd J_;
  Eigen::VectorXd r_;
  Eigen::VectorXd x0_;

  std::set<uint64_t> frame_ids_;
  std::set<uint64_t> preintegration_ids_;
  std::vector<int>   block_sizes_;
};

class MarginalizationCost : public ceres::CostFunction {
 public:
  explicit MarginalizationCost(const MarginalizationPrior& info)
    : J_(info.J_)
    , r_(info.r_)
    , x0_(info.x0_)
    , block_sizes_(info.block_sizes_) {
    set_num_residuals(static_cast<int>(r_.size()));
    mutable_parameter_block_sizes()->assign(block_sizes_.begin(),
                                            block_sizes_.end());
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

    if (state_dim != x0_.size() || J_.rows() != residual_dim
        || J_.cols() != state_dim) {
      return false;
    }

    Eigen::VectorXd dx(state_dim);
    int             col_offset = 0;
    for (int i = 0; i < num_blocks; ++i) {
      const int                         block_size = block_sizes_[i];
      Eigen::Map<const Eigen::VectorXd> x_i(params[i], block_size);
      dx.segment(col_offset,
                 block_size) = x_i - x0_.segment(col_offset, block_size);
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
        Eigen::Map<
          Eigen::
            Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
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
}  // namespace omni_slam
