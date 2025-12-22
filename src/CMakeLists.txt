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

target_sources(omni_core INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/core/utils/logger.hpp

)

# Dataset loader library
add_library(device INTERFACE

)

target_sources(device INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/device/vio_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/device/euroc_loader.cpp
)

target_link_libraries(device INTERFACE
  omni_core
)

target_compile_features(device INTERFACE cxx_std_20)


if(OMNI_BUILD_STEREO_VO)
  set(VO_SOURCES
    odometry/stereo_vo.cpp
  )

  add_executable(stereo_vo app/run_vo.cpp ${VO_SOURCES})
  target_include_directories(stereo_vo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_vo omni_core device ${OpenCV_LIBS})
endif()


if(OMNI_BUILD_LIO)
endif()

# Tests
if(OMNI_BUILD_TESTS)
  enable_testing()
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()
