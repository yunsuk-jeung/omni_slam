#include "feature_tracking/grid_detector.hpp"
#include "mapper/descriptor_extractor.hpp"
#include "utils/omni_assert.hpp"

namespace omni_slam {
namespace {

std::bitset<kDescriptorBits> to_descriptor(const cv::Mat& row) {
  std::bitset<kDescriptorBits> descriptor;
  for (size_t bit = 0; bit < kDescriptorBits; ++bit) {
    const uchar byte = row.at<uchar>(0, static_cast<int>(bit >> 3));
    if (byte & (1u << (bit & 7u))) {
      descriptor.set(bit);
    }
  }
  return descriptor;
}

}  // namespace

DescriptorExtractor::DescriptorExtractor(int            grid_rows,
                                         int            grid_cols,
                                         int            fast_threshold,
                                         int            max_per_cell,
                                         DescriptorType type)
  : grid_rows_{grid_rows}
  , grid_cols_{grid_cols}
  , fast_threshold_{fast_threshold}
  , max_per_cell_{max_per_cell} {
  switch (type) {
  case DescriptorType::kOrb:
  default:
    impl_ = cv::ORB::create();
    break;
  }
}

void DescriptorExtractor::detect_and_compute(
  const cv::Mat&                             image,
  std::vector<Eigen::Vector2d>&              uvs,
  std::vector<std::bitset<kDescriptorBits>>& descriptors) const {
  uvs.clear();
  descriptors.clear();

  std::vector<cv::KeyPoint> keypoints = detect_grid_features(image,
                                                             grid_rows_,
                                                             grid_cols_,
                                                             fast_threshold_,
                                                             {},
                                                             max_per_cell_);

  // compute() drops border keypoints and reorders, so read uvs back afterwards.
  cv::Mat descriptor_mat;
  impl_->compute(image, keypoints, descriptor_mat);

  if (descriptor_mat.empty()) {
    return;
  }

  OMNI_ASSERT(descriptor_mat.cols * 8 == static_cast<int>(kDescriptorBits));
  uvs.reserve(keypoints.size());
  descriptors.reserve(keypoints.size());
  for (int i = 0; i < descriptor_mat.rows; ++i) {
    const cv::Point2f& pt = keypoints[static_cast<size_t>(i)].pt;
    uvs.emplace_back(pt.x, pt.y);
    descriptors.push_back(to_descriptor(descriptor_mat.row(i)));
  }
}

}  // namespace omni_slam
