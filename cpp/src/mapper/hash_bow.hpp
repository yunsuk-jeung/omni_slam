#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace omni_slam {

// Hash-based bag-of-words for loop-closure retrieval; approach after
// Galvez-Lopez (DBoW) and Usenko et al. (basalt), reimplemented here.
namespace detail {

template <size_t N>
constexpr std::array<size_t, N> make_bit_permutation(uint64_t seed) {
  std::array<size_t, N> perm{};
  for (size_t i = 0; i < N; ++i) {
    perm[i] = i;
  }
  uint64_t state = seed;
  for (size_t i = N - 1; i > 0; --i) {
    state          = state * 6364136223846793005ULL + 1442695040888963407ULL;
    const size_t j = static_cast<size_t>((state >> 33) % (i + 1));
    const size_t t = perm[i];
    perm[i]        = perm[j];
    perm[j]        = t;
  }
  return perm;
}

}  // namespace detail

template <size_t N>
class HashBow {
 public:
  struct ScoredKeyframe {
    uint64_t keyframe_id = 0;
    double   score       = 0.0;
  };

  explicit HashBow(size_t num_word_bits)
    : num_word_bits_{std::min<size_t>({num_word_bits, kMaxWordBits, N})} {}

  uint32_t compute_hash(const std::bitset<N>& descriptor) const {
    uint32_t hash = 0;
    for (size_t i = 0; i < num_word_bits_; ++i) {
      if (descriptor[kBitPermutation[i]]) {
        hash |= (uint32_t{1} << i);
      }
    }
    return hash;
  }

  void compute_bow(const std::vector<std::bitset<N>>&        descriptors,
                   std::vector<std::pair<uint32_t, double>>& bow_vector) const {
    std::unordered_map<uint32_t, double> term_freq;
    term_freq.reserve(descriptors.size());
    for (const auto& descriptor : descriptors) {
      term_freq[compute_hash(descriptor)] += 1.0;
    }

    bow_vector.assign(term_freq.begin(), term_freq.end());

    double l1_sum = 0.0;
    for (const auto& [word, weight] : bow_vector) {
      l1_sum += std::abs(weight);
    }
    if (l1_sum > 0.0) {
      for (auto& [word, weight] : bow_vector) {
        weight /= l1_sum;
      }
    }
  }

  void add_to_database(
    uint64_t                                        keyframe_id,
    const std::vector<std::pair<uint32_t, double>>& bow_vector) {
    for (const auto& [word, weight] : bow_vector) {
      inverted_index_[word].push_back({keyframe_id, weight});
    }
  }

  void query(
    const std::vector<std::pair<uint32_t, double>>& bow_vector,
    size_t                                          num_results,
    std::vector<ScoredKeyframe>&                    results,
    uint64_t newest_allowed_id = std::numeric_limits<uint64_t>::max()) const {
    std::unordered_map<uint64_t, double> scores;
    for (const auto& [word, weight] : bow_vector) {
      const auto it = inverted_index_.find(word);
      if (it == inverted_index_.end()) {
        continue;
      }
      for (const auto& entry : it->second) {
        if (entry.keyframe_id > newest_allowed_id) {
          continue;
        }
        // L1 similarity: 2 - ||a - b||_1 accumulated over shared words.
        scores[entry.keyframe_id] += std::abs(weight - entry.weight)
                                     - std::abs(weight)
                                     - std::abs(entry.weight);
      }
    }

    results.clear();
    results.reserve(scores.size());
    for (const auto& [id, score] : scores) {
      results.push_back({id, -score / 2.0});
    }

    if (results.size() > num_results) {
      std::partial_sort(results.begin(),
                        results.begin() + num_results,
                        results.end(),
                        [](const ScoredKeyframe& a, const ScoredKeyframe& b) {
                          return a.score > b.score;
                        });
      results.resize(num_results);
    }
  }

 private:
  struct Entry {
    uint64_t keyframe_id = 0;
    double   weight      = 0.0;
  };

  static constexpr size_t   kMaxWordBits     = 32;
  static constexpr uint64_t kPermutationSeed = 0x9E3779B97F4A7C15ULL;
  static constexpr std::array<size_t, N> kBitPermutation =
    detail::make_bit_permutation<N>(kPermutationSeed);

  size_t                                           num_word_bits_;
  std::unordered_map<uint32_t, std::vector<Entry>> inverted_index_;
};

}  // namespace omni_slam
