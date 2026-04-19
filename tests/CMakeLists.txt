# Core module tests
add_executable(core_tests
  core/test_logger.cpp
)

target_link_libraries(core_tests PRIVATE
  omni_core
  GTest::gtest
  GTest::gtest_main
)

add_test(NAME CoreTests COMMAND core_tests)

# Optimizer tests
add_executable(optimizer_tests
  optimizer/test_imu_preintegration_cost.cpp
  ${CMAKE_SOURCE_DIR}/src/odometry/imu_preintegration.cpp
)

target_link_libraries(optimizer_tests PRIVATE
  omni_utils
  GTest::gtest
  GTest::gtest_main
)

add_test(NAME OptimizerTests COMMAND optimizer_tests)
