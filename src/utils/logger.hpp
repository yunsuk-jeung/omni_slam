#pragma once

#include <chrono>
#include <filesystem>
#include <format>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "utils/fmt_eigen.hpp"

namespace omni_slam {

class Logger {
 public:
  static void init(bool enable_file_logging = true) {
    if (!logger_) {
      if (enable_file_logging) {
        // Keep logs in project-root/logs regardless of process working
        // directory.
#ifdef OMNI_SLAM_SOURCE_DIR
        const auto log_dir = std::filesystem::path(OMNI_SLAM_SOURCE_DIR)
                             / "logs";
#else
        const auto log_dir = std::filesystem::path("logs");
#endif
        std::filesystem::create_directories(log_dir);

        // Generate filename with current timestamp
        auto              now  = std::chrono::system_clock::now();
        auto              time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "omni_slam_"
           << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".log";
        const std::string log_file = (log_dir / ss.str()).string();

        // Create sinks for both console and file output
        auto console_sink =
          std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink =
          std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);

        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
        logger_ = std::make_shared<spdlog::logger>("omni_slam",
                                                   sinks.begin(),
                                                   sinks.end());
        logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

        logger_->info("Logging to file: {}", log_file);
      }
      else {
        logger_ = spdlog::default_logger();
      }
      spdlog::set_default_logger(logger_);
    }
  }

  template <typename... Args>
  static void debug(const char* fmt, Args&&... args) {
    init();
    logger_->debug(fmt, args...);
  }

  template <typename... Args>
  static void info(const char* fmt, Args&&... args) {
    init();
    logger_->info(fmt, args...);
  }

  template <typename... Args>
  static void warn(const char* fmt, Args&&... args) {
    init();
    logger_->warn(fmt, args...);
  }

  template <typename T, typename... Args>
  static void error(const char* file, int line, const T& fmt, Args&&... args) {
    init();
    const std::string_view fmt_view(fmt);
    if constexpr (sizeof...(args) == 0) {
      logger_->error("[{}:{}] {}", file, line, fmt_view);
    }
    else {
      auto arg_tuple =
        std::tuple<std::decay_t<Args>...>(std::forward<Args>(args)...);
      auto format_args = std::apply(
        [](auto&... unpacked) { return std::make_format_args(unpacked...); },
        arg_tuple);
      logger_->error("[{}:{}] {}",
                     file,
                     line,
                     std::vformat(fmt_view, format_args));
    }
  }

  static inline const char* extract_file_name(const char* path) {
    if (!path) {
      return "";
    }
    const char* last_slash = path;
    for (const char* p = path; *p != '\0'; ++p) {
      if (*p == '/' || *p == '\\') {
        last_slash = p + 1;
      }
    }
    return last_slash;
  }

 private:
  static inline std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace omni_slam

#define LogD(fmt, ...) omni_slam::Logger::debug(fmt, ##__VA_ARGS__);
#define LogI(fmt, ...) omni_slam::Logger::info(fmt, ##__VA_ARGS__);
#define LogW(fmt, ...) omni_slam::Logger::warn(fmt, ##__VA_ARGS__);
#define LogE(fmt, ...)                                                         \
  omni_slam::Logger::error(omni_slam::Logger::extract_file_name(__FILE__),     \
                           __LINE__,                                           \
                           fmt,                                                \
                           ##__VA_ARGS__);
#define DEBUG_POINT() LogE("THIS Line is for debugging");
