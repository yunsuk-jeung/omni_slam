#pragma once

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace omni_slam {

class Logger {
public:
  static void Init(bool enable_file_logging = false) {
    if (!logger_) {
      if (enable_file_logging) {
        // Create logs directory if it doesn't exist
        std::filesystem::create_directories("logs");

        // Generate filename with current timestamp
        auto              now  = std::chrono::system_clock::now();
        auto              time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "logs/omni_slam_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
           << ".log";
        std::string log_file = ss.str();

        // Create sinks for both console and file output
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink    = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file,
                                                                             true);

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
  static void Debug(const char* fmt, Args&&... args) {
    Init();
    logger_->debug(fmt::runtime(fmt), std::forward<Args>(args)...);
  }
  // static void Debug(std::string_view message) {
  //   Init();
  //   logger_->debug("{}", message);
  // }

  template <typename... Args>
  static void Info(const char* fmt, Args&&... args) {
    Init();
    logger_->info(fmt::runtime(fmt), std::forward<Args>(args)...);
  }
  // static void Info(std::string_view message) {
  //   Init();
  //   logger_->info("{}", message);
  // }

  template <typename... Args>
  static void Warn(const char* fmt, Args&&... args) {
    Init();
    logger_->warn(fmt::runtime(fmt), std::forward<Args>(args)...);
  }
  // static void Warn(std::string_view message) {
  //   Init();
  //   logger_->warn("{}", message);
  // }

  template <typename T, typename... Args>
  static void Error(const char* file, int line, const T& fmt, Args&&... args) {
    Init();
    if constexpr (sizeof...(args) == 0) {
      logger_->error("[{}:{}] {}", file, line, fmt);
    }
    else {
      logger_->error("[{}:{}] {}",
                     file,
                     line,
                     fmt::format(fmt::runtime(fmt), std::forward<Args>(args)...));
    }
  }

  static inline const char* extractFileName(const char* path) {
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

#define LogD(fmt, ...) omni_slam::Logger::Debug(fmt, ##__VA_ARGS__);
#define LogI(fmt, ...) omni_slam::Logger::Info(fmt, ##__VA_ARGS__);
#define LogW(fmt, ...) omni_slam::Logger::Warn(fmt, ##__VA_ARGS__);
#define LogE(fmt, ...)                                                                   \
  omni_slam::Logger::Error(omni_slam::Logger::extractFileName(__FILE__),                 \
                           __LINE__,                                                     \
                           fmt,                                                          \
                           ##__VA_ARGS__);
#define DEBUG_POINT() LogE("THIS Line is for debugging");
