#pragma once

#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "utils/logger.hpp"

namespace omni_slam {

class Timer {
 public:
  using Clock = std::chrono::steady_clock;

  Timer() { start(); }

  void start() { start_time_ = Clock::now(); }

  double stop_ms() const {
    const auto                                      end_time = Clock::now();
    const std::chrono::duration<double, std::milli> ms = end_time - start_time_;
    return ms.count();
  }

 private:
  Clock::time_point start_time_;
};

struct TimerElement {
  Timer  timer;
  int    call_count = 0;
  double total_time = 0.0;
  double min_time   = std::numeric_limits<double>::max();
  double max_time   = std::numeric_limits<double>::lowest();
  double last_time  = 0.0;
};

class Statistics {
 public:
  static void start_timer(const std::string& name) {
    get_or_create(name).timer.start();
  }

  static double stop_timer(const std::string& name) {
    auto&        elem = get_or_create(name);
    const double ms   = elem.timer.stop_ms();
    elem.last_time    = ms;
    elem.call_count++;
    elem.total_time += ms;
    if (ms < elem.min_time) {
      elem.min_time = ms;
    }
    if (ms > elem.max_time) {
      elem.max_time = ms;
    }
    return ms;
  }

  static double mean_time(const std::string& name) {
    const auto it = statistics().find(name);
    if (it == statistics().end() || it->second.call_count == 0) {
      return 0.0;
    }
    return it->second.total_time / it->second.call_count;
  }

  static void report(const std::vector<std::string>& ids) {
    report_header();
    for (const auto& id : ids) {
      report_one(id);
    }
  }

  static void report_all() {
    report_header();
    if (order().empty()) {
      LogW("No timers recorded.");
      return;
    }
    for (const auto& name : order()) {
      report_one(name);
    }
  }

 private:
  static TimerElement& get_or_create(const std::string& name) {
    auto& stats = statistics();
    auto  it    = stats.find(name);
    if (it != stats.end()) {
      return it->second;
    }

    auto [inserted_it, inserted] = stats.emplace(name, TimerElement{});
    if (inserted) {
      order().push_back(name);
    }
    return inserted_it->second;
  }

  static std::map<std::string, TimerElement>& statistics() {
    static std::map<std::string, TimerElement> stats;
    return stats;
  }

  static std::vector<std::string>& order() {
    static std::vector<std::string> names;
    return names;
  }

  static void report_header() {
    LogI("Timer statistics (ms):");
    LogI("{:<24} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10}",
         "Name",
         "Calls",
         "Latest",
         "Mean",
         "Min",
         "Max",
         "Total");
  }

  static void report_one(const std::string& name) {
    const auto it = statistics().find(name);
    if (it == statistics().end()) {
      LogW("Timer '{}' not found.", name);
      return;
    }

    const auto&  e    = it->second;
    const double mean = (e.call_count == 0) ? 0.0
                                            : (e.total_time / e.call_count);
    const double min  = (e.call_count == 0) ? 0.0 : e.min_time;
    const double max  = (e.call_count == 0) ? 0.0 : e.max_time;

    LogI("{:<24} {:>8} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.3f}",
         name,
         e.call_count,
         e.last_time,
         mean,
         min,
         max,
         e.total_time);
  }
};

class ScopedTimer {
 public:
  explicit ScopedTimer(std::string name)
    : name_(std::move(name)) {
    Statistics::start_timer(name_);
  }

  ~ScopedTimer() { Statistics::stop_timer(name_); }

 private:
  std::string name_;
};

}  // namespace omni_slam
