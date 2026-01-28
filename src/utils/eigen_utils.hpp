
#pragma once

#include <bitset>
#include <cstdint>
#include <Eigen/Dense>

namespace Eigen {
using Matrix66d = Eigen::Matrix<double, 6, 6>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Matrix62d = Eigen::Matrix<double, 6, 2>;
using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen