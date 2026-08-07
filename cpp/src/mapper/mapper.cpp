#include "mapper/mapper.hpp"
#include "utils/logger.hpp"

namespace omni_slam {
namespace {
constexpr size_t kHashWordBits = 24;
}  // namespace

Mapper::Mapper()
  : hash_bow_{kHashWordBits} {}

Mapper::~Mapper() {
  shutdown();
}

void Mapper::run() {
  Logger::info("Running Mapper");
  running_.store(true, std::memory_order_release);
  mapper_thread_ = std::thread(&Mapper::mapper_loop, this);
}

void Mapper::shutdown() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  Logger::info("Shutting down Mapper");
  input_queue_.close();
  if (mapper_thread_.joinable()) {
    mapper_thread_.join();
  }
}

void Mapper::mapper_loop() {
  KeyframeMargData input;
  while (running_.load(std::memory_order_acquire)) {
    if (!input_queue_.wait()) {
      break;
    }
    if (!input_queue_.try_pop(input)) {
      continue;
    }
    process(input);
  }
}

void Mapper::process(const KeyframeMargData& input) {
  // TODO(A): loop detection + verification, global BA/pose-graph.

  // Recover NFR factors from the prior and register keyframe poses.
  nfr_optimizer_.add_marg_data(input.prior);

  // Index the leaving keyframes for loop-closure retrieval.
  for (const auto& keyframe : input.keyframes) {
    if (keyframe.images.empty()) {
      continue;
    }
    std::vector<Eigen::Vector2d>              uvs;
    std::vector<std::bitset<kDescriptorBits>> descriptors;
    descriptor_extractor_.detect_and_compute(keyframe.images.front(),
                                             uvs,
                                             descriptors);

    std::vector<std::pair<uint32_t, double>> bow_vector;
    hash_bow_.compute_bow(descriptors, bow_vector);
    hash_bow_.add_to_database(keyframe.keyframe_id, bow_vector);

    Logger::info("Mapper added keyframe {} to database ({} descriptors)",
                 keyframe.keyframe_id,
                 descriptors.size());
  }
}

}  // namespace omni_slam
