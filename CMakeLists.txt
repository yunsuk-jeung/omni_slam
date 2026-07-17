cmake_minimum_required(VERSION 3.20)

# --- Host-dependent compiler selection ---------------------------------------
# Must run before project(). Skipped if the compiler is already pinned via a
# preset, -DCMAKE_CXX_COMPILER=..., or the CXX environment variable.
if(NOT DEFINED CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
  if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    # Ubuntu: prefer a modern GCC with full C++20 support.
    find_program(_OMNI_CXX NAMES g++-13 g++-14 g++)
    find_program(_OMNI_CC  NAMES gcc-13 gcc-14 gcc)
  elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    # macOS: use Clang (Homebrew LLVM if on PATH, otherwise AppleClang).
    find_program(_OMNI_CXX NAMES clang++)
    find_program(_OMNI_CC  NAMES clang)
  endif()
  if(_OMNI_CXX)
    set(CMAKE_CXX_COMPILER "${_OMNI_CXX}" CACHE FILEPATH "C++ compiler")
  endif()
  if(_OMNI_CC)
    set(CMAKE_C_COMPILER "${_OMNI_CC}" CACHE FILEPATH "C compiler")
  endif()
  unset(_OMNI_CXX CACHE)
  unset(_OMNI_CC CACHE)
endif()

project(omni_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json and symlink it to the source root for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(CREATE_LINK "${CMAKE_BINARY_DIR}/compile_commands.json"
     "${CMAKE_SOURCE_DIR}/compile_commands.json" SYMBOLIC)

# Build options
option(OMNI_BUILD_STEREO_VO "Build Stereo Visual Odometry pipeline" ON)
option(OMNI_BUILD_STEREO_VIO "Build Stereo Visual-Inertial Odometry pipeline" OFF)
option(OMNI_BUILD_LIO "Build LiDAR-Inertial Odometry pipeline" OFF)
option(OMNI_BUILD_LOOP "Build Loop Closure module" ON)
option(OMNI_BUILD_VIEWER "Build Vulkan viewer app" OFF)
option(OMNI_BUILD_TESTS "Build tests" OFF)

# Set installation paths based on build type
# For RelWithDebInfo, use Release libraries (no need for debug symbols in external deps)
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
if(BUILD_TYPE_LOWER STREQUAL "debug")
  set(DEPS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/debug")
else()
  # Use Release libraries for both Release and RelWithDebInfo builds
  set(DEPS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/release")
endif()

# Map RelWithDebInfo to Release for all imported targets
# This tells CMake to use Release configuration when RelWithDebInfo is not available
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release "")

# Find installed dependencies
set(CMAKE_PREFIX_PATH "${DEPS_INSTALL_DIR}" ${CMAKE_PREFIX_PATH})

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(GTest REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui)
find_package(Sophus REQUIRED)
find_package(nlohmann_json REQUIRED)

# Source tree (build options live in src/CMakeLists.txt)
add_subdirectory(cpp/src)
add_subdirectory(apps)
