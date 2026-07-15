#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace omni_slam {

template <typename T>
class ConcurrentQueue {
 public:
  explicit ConcurrentQueue(size_t max_size = 0)
    : max_size_{max_size} {}

  void push(T value) {
    std::scoped_lock lock(mutex_);
    queue_.push_back(std::move(value));
    if (max_size_ > 0 && queue_.size() > max_size_) {
      queue_.pop_front();
    }
  }

  bool try_pop(T& out) {
    std::scoped_lock lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void push_front(T value) {
    std::scoped_lock lock(mutex_);
    queue_.push_front(std::move(value));
  }

  bool try_peek(T& out) const {
    std::scoped_lock lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    out = queue_.front();
    return true;
  }

  size_t size() const {
    std::scoped_lock lock(mutex_);
    return queue_.size();
  }

 private:
  mutable std::mutex mutex_;
  std::deque<T>      queue_;
  size_t             max_size_;
};

}  // namespace omni_slam
