#pragma once

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <vector>

namespace omni_slam {

class Logger {
public:
  static void Init(bool enable_file_logging = false) {
    if (!logger_) {
      if (enable_file_logging) {
        //Create logs directory if it doesn't exist
        std::filesystem::create_directories("logs");

        //Generate filename with current timestamp
        auto              now  = std::chrono::system_clock::now();
        auto              time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "logs/omni_slam_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
           << ".log";
        std::string log_file = ss.str();

        //Create sinks for both console and file output
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

  static void Debug(const std::string& message) {
    Init();
    logger_->debug(message);
  }

  static void Info(const std::string& message) {
    Init();
    logger_->info(message);
  }

  static void Warn(const std::string& message) {
    Init();
    logger_->warn(message);
  }

  static void Error(const std::string& message) {
    Init();
    logger_->error(message);
  }

private:
  static inline std::shared_ptr<spdlog::logger> logger_;
};

}  //namespace omni_slam
