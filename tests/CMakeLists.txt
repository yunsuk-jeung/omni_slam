# Core module tests
add_executable(core_tests
  core/test_timestamp.cpp
  core/test_types.cpp
  core/test_logger.cpp
)

target_link_libraries(core_tests PRIVATE
  omni_core
  GTest::gtest
  GTest::gtest_main
)

add_test(NAME CoreTests COMMAND core_tests)
