# Core library (always built)
add_library(omni_utils INTERFACE)

target_include_directories(omni_utils INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(omni_utils INTERFACE
  Eigen3::Eigen
  Sophus::Sophus
  spdlog::spdlog
  TBB::tbb
)

target_compile_features(omni_utils INTERFACE cxx_std_20)

# Dataset loader library
add_library(omni_device INTERFACE)

target_sources(omni_device INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/device/vio_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/device/euroc_loader.cpp
)

target_link_libraries(omni_device INTERFACE
  omni_utils
)

target_compile_features(omni_device INTERFACE cxx_std_20)

# Dataset loader library
add_library(omni_optical_flow INTERFACE)

target_sources(omni_optical_flow INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/optical_flow/optical_flow.cpp
)

target_link_libraries(omni_optical_flow INTERFACE
  omni_utils
)

target_compile_features(omni_optical_flow INTERFACE cxx_std_20)

if(OMNI_BUILD_STEREO_VO)
  set(VO_SOURCES
    odometry/stereo_vo.cpp
  )

  add_executable(stereo_vo app/run_svo.cpp ${VO_SOURCES})
  target_include_directories(stereo_vo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_vo omni_utils omni_device omni_optical_flow ${OpenCV_LIBS})
endif()


if(OMNI_BUILD_LIO)
endif()

# Tests
if(OMNI_BUILD_TESTS)
  enable_testing()
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()
