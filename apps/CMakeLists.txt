# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
  https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

if(OMNI_BUILD_STEREO_VO)
  add_executable(stereo_vo
    ${CMAKE_SOURCE_DIR}/apps/run_vo.cpp
  )
  target_include_directories(stereo_vo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_vo omni_utils omni_device svo ${OpenCV_LIBS} rerun_sdk)
endif()
