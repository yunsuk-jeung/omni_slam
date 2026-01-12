#!/bin/bash

# Build and install third-party dependencies for omni_slam
# Usage: ./build_dependencies.sh [debug|release|relwithdebinfo]

set -e  # Exit on error

# Get build type (default: relwithdebinfo)
BUILD_TYPE=${1:-relwithdebinfo}

# Convert to proper CMake build type
case "$BUILD_TYPE" in
    debug)
        CMAKE_BUILD_TYPE="Debug"
        INSTALL_DIR="libs/debug"
        ;;
    release)
        CMAKE_BUILD_TYPE="Release"
        INSTALL_DIR="libs/release"
        ;;
    relwithdebinfo)
        CMAKE_BUILD_TYPE="RelWithDebInfo"
        INSTALL_DIR="libs/relwithdebinfo"
        ;;
    *)
        echo "Invalid build type: $BUILD_TYPE"
        echo "Usage: $0 [debug|release|relwithdebinfo]"
        exit 1
        ;;
esac

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
THIRD_PARTY_DIR="$PROJECT_ROOT/third_party"
INSTALL_PREFIX="$PROJECT_ROOT/$INSTALL_DIR"
BUILD_DIR="$PROJECT_ROOT/build/deps-$BUILD_TYPE"
PREFIX_PATH="$CMAKE_PREFIX_PATH"
if [ -n "$PREFIX_PATH" ]; then
    PREFIX_PATH="$PREFIX_PATH;$INSTALL_PREFIX"
else
    PREFIX_PATH="$INSTALL_PREFIX"
fi

echo "=================================================="
echo "Building third-party dependencies"
echo "  Build Type: $CMAKE_BUILD_TYPE"
echo "  Install To: $INSTALL_PREFIX"
echo "  Cmake prefix: $INSTALL_PREFIX"
echo "  Build Dir:  $BUILD_DIR"
echo "=================================================="

# Create build and install directories
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_PREFIX"

# Number of parallel jobs
NPROC=$(nproc)

# Function to build and install a library
build_library() {
    local name=$1
    local source_dir=$2
    local cmake_args=$3

    echo ""
    echo "Building $name..."
    echo "----------------------------------------"

    local lib_build_dir="$BUILD_DIR/$name"
    mkdir -p "$lib_build_dir"
    cd "$lib_build_dir"

    cmake "$source_dir" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE" \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX_PATH" \
        $cmake_args

    cmake --build . -j$NPROC
    cmake --install .

    echo "$name installed successfully!"
}

# # 1. Eigen
# build_library "eigen" "$THIRD_PARTY_DIR/eigen" \
#     "-DEIGEN_BUILD_DOC=OFF \
#      -DBUILD_TESTING=OFF \
#      -DEIGEN_BUILD_PKGCONFIG=OFF"

# # 2. Sophus (depends on Eigen)
# build_library "sophus" "$THIRD_PARTY_DIR/sophus" \
#     "-DBUILD_SOPHUS_TESTS=OFF \
#      -DBUILD_SOPHUS_EXAMPLES=OFF"

# 3. nlohmann_json
build_library "nlohmann_json" "$THIRD_PARTY_DIR/json" \
    "-DJSON_BuildTests=OFF \
     -DJSON_Install=ON"

# # 4. spdlog
# build_library "spdlog" "$THIRD_PARTY_DIR/spdlog" \
#     "-DSPDLOG_BUILD_EXAMPLE=OFF \
#      -DSPDLOG_USE_STD_FORMAT=ON \
#      -DSPDLOG_BUILD_SHARED=ON \
#      -DSPDLOG_BUILD_TESTS=OFF"

# # 5. TBB
# build_library "tbb" "$THIRD_PARTY_DIR/oneTBB" \
#     "-DTBB_TEST=OFF \
#      -DTBB_EXAMPLES=OFF \
#      -DTBB_STRICT=OFF"

# # 6. GoogleTest
# build_library "googletest" "$THIRD_PARTY_DIR/googletest" \
#     "-DBUILD_GMOCK=OFF \
#      -DINSTALL_GTEST=ON"

# # 7. Ceres Solver (depends on Eigen)
# build_library "ceres-solver" "$THIRD_PARTY_DIR/ceres-solver" \
#     "-DBUILD_TESTING=OFF \
#      -DBUILD_EXAMPLES=OFF \
#      -DBUILD_BENCHMARKS=OFF \
#      -DPROVIDE_UNINSTALL_TARGET=OFF"

# sudo apt install -y \
#   libgtk-3-dev \
#   pkg-config

# # 8. OpenCV
# build_library "opencv" "$THIRD_PARTY_DIR/opencv" \
#     "-DBUILD_TESTS=OFF \
#      -DBUILD_PERF_TESTS=OFF \
#      -DBUILD_EXAMPLES=OFF \
#      -DBUILD_opencv_apps=OFF \
#      -DBUILD_DOCS=OFF \
#      -DWITH_GTK=ON \
#      -DWITH_QT=OFF \
#      -DWITH_TBB=ON \
#      -DWITH_CUDA=OFF \
#      -DBUILD_opencv_world=ON"

echo ""
echo "=================================================="
echo "All dependencies built and installed successfully!"
echo "Install location: $INSTALL_PREFIX"
echo "=================================================="
