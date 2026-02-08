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

  double stopMs() const {
    const auto end_time = Clock::now();
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
  static void startTimer(const std::string& name) {
    getOrCreate(name).timer.start();
  }

  static double stopTimer(const std::string& name) {
    auto& elem = getOrCreate(name);
    const double ms = elem.timer.stopMs();
    elem.last_time = ms;
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

  static double meanTime(const std::string& name) {
    const auto it = statistics().find(name);
    if (it == statistics().end() || it->second.call_count == 0) {
      return 0.0;
    }
    return it->second.total_time / it->second.call_count;
  }

  static void report(const std::vector<std::string>& ids) {
    reportHeader();
    for (const auto& id : ids) {
      reportOne(id);
    }
  }

  static void reportAll() {
    reportHeader();
    if (order().empty()) {
      LogW("No timers recorded.");
      return;
    }
    for (const auto& name : order()) {
      reportOne(name);
    }
  }

private:
  static TimerElement& getOrCreate(const std::string& name) {
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

  static void reportHeader() {
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

  static void reportOne(const std::string& name) {
    const auto it = statistics().find(name);
    if (it == statistics().end()) {
      LogW("Timer '{}' not found.", name);
      return;
    }

    const auto& e = it->second;
    const double mean = (e.call_count == 0) ? 0.0 : (e.total_time / e.call_count);
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
  explicit ScopedTimer(std::string name) : name_(std::move(name)) {
    Statistics::startTimer(name_);
  }

  ~ScopedTimer() { Statistics::stopTimer(name_); }

private:
  std::string name_;
};

}  // namespace omni_slam
