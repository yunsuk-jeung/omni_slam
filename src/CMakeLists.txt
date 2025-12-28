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
  nlohmann_json::nlohmann_json
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
add_library(omni_camera_model INTERFACE)

target_link_libraries(omni_camera_model INTERFACE
  omni_utils
)

target_compile_features(omni_camera_model INTERFACE cxx_std_20)

if(OMNI_BUILD_STEREO_VO)

  add_library(svo SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/optical_flow/optical_flow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config/svo_config.cpp
  )
  target_include_directories(svo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(svo omni_utils omni_camera_model ${OpenCV_LIBS})
  target_compile_features(svo INTERFACE cxx_std_20)

  set(VO_SOURCES
    odometry/stereo_vo.cpp
    database/Frame.cpp
  )

  add_executable(stereo_vo app/run_svo.cpp ${VO_SOURCES})
  target_include_directories(stereo_vo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_vo omni_utils omni_device svo ${OpenCV_LIBS})
endif()


if(OMNI_BUILD_LIO)
endif()

# Tests
if(OMNI_BUILD_TESTS)
  enable_testing()
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()
