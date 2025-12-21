# Core library (always built)
add_library(omni_core INTERFACE)

target_include_directories(omni_core INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(omni_core INTERFACE
  Eigen3::Eigen
  Sophus::Sophus
  spdlog::spdlog
  TBB::tbb
)

target_compile_features(omni_core INTERFACE cxx_std_20)

# target_sources(omni_core INTERFACE
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/types/measurement.hpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/types/state.hpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/types/feature.hpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/time/timestamp.hpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/utils/logger.hpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/core/utils/config.hpp
# )


if(OMNI_BUILD_VO)
  set(VO_SOURCES
    odometry/vo_pipeline.cpp
  )

  add_executable(vo app/run_vo.cpp ${VO_SOURCES})
  target_link_libraries(vo omni_core)
endif()


if(OMNI_BUILD_LIO)
endif()

# Tests
if(OMNI_BUILD_TESTS)
  enable_testing()
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()
