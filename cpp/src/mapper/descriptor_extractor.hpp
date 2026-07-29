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
  explicit DescriptorExtractor(size_t         max_features = 1000,
                               DescriptorType type         = DescriptorType::kOrb);

  void detect_and_compute(
    const cv::Mat&                             image,
    std::vector<Eigen::Vector2d>&              uvs,
    std::vector<std::bitset<kDescriptorBits>>& descriptors) const;

 private:
  cv::Ptr<cv::Feature2D> impl_;
};

}  // namespace omni_slam
