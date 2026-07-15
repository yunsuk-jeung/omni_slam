#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace omni_slam {

// Mutex-based FIFO shared between producer and consumer threads.
// max_size 0 means unbounded; otherwise push drops the oldest element
// once the queue is full.
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

  // Pops only when the front element satisfies pred.
  template <typename Pred>
  bool try_pop_if(T& out, Pred pred) {
    std::scoped_lock lock(mutex_);
    if (queue_.empty() || !pred(queue_.front())) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  // Copies the front element without removing it.
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
