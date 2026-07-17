#pragma once

#include <format>
#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Dense>

namespace omni_slam {

struct EigenFormat {
  std::string value;
};

template <typename Derived>
inline EigenFormat fmt_eigen(const Eigen::MatrixBase<Derived>& mat) {
  static constexpr int eigenPrecision = 5;
  const bool           is_col_vec     = (mat.cols() == 1);
  const bool           is_row_vec     = (mat.rows() == 1);
  const bool           is_vector      = is_col_vec || is_row_vec;

  static constexpr const char* kMatrixRowIndent =
    "                                         [";
  static constexpr const char* kMatrixRowDivider =
    "-------------------------------------------\n";

  Eigen::IOFormat format(eigenPrecision,
                         0,
                         ", ",
                         is_vector ? "\n" : ",\n",
                         is_vector ? "[" : kMatrixRowIndent,
                         "]",
                         is_vector ? "" : kMatrixRowDivider,
                         "");

  std::stringstream ss;
  ss << std::fixed << std::setprecision(eigenPrecision);
  if (is_col_vec) {
    ss << mat.transpose().format(format);
  }
  else {
    ss << mat.derived().format(format);
  }

  return EigenFormat{ss.str()};
}

}  // namespace omni_slam

namespace std {

template <>
struct formatter<omni_slam::EigenFormat, char> : formatter<string_view, char> {
  template <typename FormatContext>
  auto format(const omni_slam::EigenFormat& value, FormatContext& ctx) const
    -> decltype(ctx.out()) {
    return formatter<string_view, char>::format(value.value, ctx);
  }
};

}  // namespace std
