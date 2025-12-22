#include "core/utils/logger.hpp"
#include <gtest/gtest.h>

namespace omni_slam {

TEST(LoggerTest, BasicLogging) {
  // Test that logger doesn't crash
  Logger::info("Test info message");
  Logger::debug("Test debug message");
  Logger::warn("Test warning message");
  Logger::error("Test error message");

  SUCCEED();
}

TEST(LoggerTest, Initialization) {
  Logger::init();
  Logger::info("Logger initialized");

  SUCCEED();
}

}  // namespace omni_slam
