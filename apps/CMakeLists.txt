# Download the rerun_sdk
include(FetchContent)
if(APPLE AND NOT DEFINED RERUN_DOWNLOAD_AND_BUILD_ARROW)
  # Prefer system Arrow on macOS to avoid long builds from source.
  set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system Arrow on macOS")
endif()

# Force external dependencies to build in Release mode
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
if(BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
  set(CMAKE_BUILD_TYPE_SAVED "${CMAKE_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for external dependencies" FORCE)
endif()

FetchContent_Declare(rerun_sdk URL
  https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Restore original build type for project code
if(BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
  set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE_SAVED}" CACHE STRING "Build type" FORCE)
  unset(CMAKE_BUILD_TYPE_SAVED)
endif()

if(OMNI_BUILD_STEREO_VO)
  add_executable(stereo_svo
    ${CMAKE_SOURCE_DIR}/apps/run_svo.cpp
  )
  target_include_directories(stereo_svo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_svo omni_utils omni_device svo ${OpenCV_LIBS} rerun_sdk)
  target_compile_options(stereo_svo PRIVATE
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:RelWithDebInfo>:-O2>
  )
endif()
