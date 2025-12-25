#include "optical_flow.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace omni_slam {

void OpticalFlow::Run(std::atomic<bool>& running) {
  std::array<cv::Mat, 2> frames;
  while (running.load(std::memory_order_acquire)) {
    if (!in_queue_.try_pop(frames)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (frames[0].empty()) {
      continue;
    }
    if (frames[1].empty()) {
      continue;
    }

    cv::Mat left_gray;
    if (frames[0].channels() == 3) {
      cv::cvtColor(frames[0], left_gray, cv::COLOR_BGR2GRAY);
    }
    else {
      left_gray = frames[0];
    }

    cv::Mat right_gray;
    if (frames[1].channels() == 3) {
      cv::cvtColor(frames[1], right_gray, cv::COLOR_BGR2GRAY);
    }
    else {
      right_gray = frames[1];
    }

    if (!has_prev_) {
      prev_left_gray_  = left_gray;
      prev_right_gray_ = right_gray;
      has_prev_        = true;
      continue;
    }

    auto curr_result = std::make_shared<TrackingResult>();

    prev_left_gray_  = left_gray.clone();
    prev_right_gray_ = right_gray.clone();
    prev_result_     = curr_result;
    out_queue_.push(curr_result);
  }
}
}  // namespace omni_slam
