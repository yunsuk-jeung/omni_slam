cmake_minimum_required(VERSION 3.20)
project(omni_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(OMNI_BUILD_VO "Build Visual Odometry pipeline" ON)
option(OMNI_BUILD_VIO "Build Visual-Inertial Odometry pipeline" OFF)
option(OMNI_BUILD_LIO "Build LiDAR-Inertial Odometry pipeline" OFF)
option(OMNI_BUILD_LOOP "Build Loop Closure module" ON)
option(OMNI_BUILD_TESTS "Build tests" OFF)

# Set installation paths based on build type
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
if(BUILD_TYPE_LOWER STREQUAL "debug")
  set(DEPS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/debug")
elseif(BUILD_TYPE_LOWER STREQUAL "release")
  set(DEPS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/release")
else()
  set(DEPS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/libs/relwithdebinfo")
endif()

# Find installed dependencies
set(CMAKE_PREFIX_PATH "${DEPS_INSTALL_DIR}" ${CMAKE_PREFIX_PATH})

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)
find_package(GTest REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui)
find_package(Sophus REQUIRED)

# Source tree (build options live in src/CMakeLists.txt)
add_subdirectory(src)
