#pragma once

#include <cstdlib>

#include "utils/logger.hpp"

#ifndef OMNI_DISABLE_ASSERT
#define OMNI_ASSERT(result)                                                              \
  ::omni_slam::OmniAssert::Assert(!!(result), __FILE__, __LINE__, __FUNCTION__)
#define OMNI_ASSERT_MESSAGE(result, message)                                             \
  ::omni_slam::OmniAssert::Assert(!!(result), (message), __FILE__, __LINE__, __FUNCTION__)
#else
#define OMNI_ASSERT(result) ((void)0)
#define OMNI_ASSERT_MESSAGE(result, message) ((void)0)
#endif

namespace omni_slam {

class OmniAssert {
public:
  static inline void Assert(bool        result,
                            const char* file,
                            int         line,
                            const char* function) {
    if (result) {
      return;
    }
    Logger::Error(Logger::extractFileName(file),
                  line,
                  "Assertion failed in {}",
                  function);
    std::abort();
  }

  static inline void Assert(bool        result,
                            const char* message,
                            const char* file,
                            int         line,
                            const char* function) {
    if (result) {
      return;
    }
    Logger::Error(Logger::extractFileName(file),
                  line,
                  "Assertion failed in {} by {}",
                  function,
                  (message ? message : ""));
    std::abort();
  }
};

}  // namespace omni_slam
