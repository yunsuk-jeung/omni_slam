# Core module tests
add_executable(core_tests
  core/test_logger.cpp
)

target_link_libraries(core_tests PRIVATE
  omni_utils
  GTest::gtest
  GTest::gtest_main
)

add_test(NAME CoreTests COMMAND core_tests)

# Optimizer tests
add_executable(optimizer_tests
  optimizer/test_cost_function_autodiff.cpp
  optimizer/test_imu_preintegration_cost.cpp
  optimizer/test_marginalization_cost.cpp
  ${CMAKE_SOURCE_DIR}/cpp/src/odometry/imu_preintegration.cpp
)

target_link_libraries(optimizer_tests PRIVATE
  omni_utils
  GTest::gtest
  GTest::gtest_main
)

add_test(NAME OptimizerTests COMMAND optimizer_tests)
