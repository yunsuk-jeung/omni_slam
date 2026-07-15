#pragma once

#include <condition_variable>
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
    {
      std::scoped_lock lock(mutex_);
      queue_.push_back(std::move(value));
      if (max_size_ > 0 && queue_.size() > max_size_) {
        queue_.pop_front();
      }
    }
    cv_.notify_one();
  }

  bool wait() const {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [&] { return closed_ || !queue_.empty(); });
    return !queue_.empty();
  }

  void close() {
    {
      std::scoped_lock lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
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
  mutable std::mutex              mutex_;
  mutable std::condition_variable cv_;
  std::deque<T>                   queue_;
  size_t                          max_size_;
  bool                            closed_ = false;
};

}  // namespace omni_slam
