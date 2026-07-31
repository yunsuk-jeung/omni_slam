#pragma once

#include <bitset>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace omni_slam {

inline constexpr size_t kDescriptorBits = 256;

enum class DescriptorType { kOrb };

class DescriptorExtractor {
 public:
  DescriptorExtractor(int            grid_rows      = 16,
                      int            grid_cols      = 16,
                      int            fast_threshold = 20,
                      int            max_per_cell   = 2,
                      DescriptorType type           = DescriptorType::kOrb);

  void detect_and_compute(
    const cv::Mat&                             image,
    std::vector<Eigen::Vector2d>&              uvs,
    std::vector<std::bitset<kDescriptorBits>>& descriptors) const;

 private:
  int                    grid_rows_;
  int                    grid_cols_;
  int                    fast_threshold_;
  int                    max_per_cell_;
  cv::Ptr<cv::Feature2D> impl_;
};

}  // namespace omni_slam
