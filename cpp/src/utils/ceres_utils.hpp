#pragma once

#include <Eigen/Dense>
#include <ceres/crs_matrix.h>

namespace omni_slam::CeresUtil {

inline bool transpose_crs_matrix(const ceres::CRSMatrix& J,
                                 ceres::CRSMatrix&       Jt) {
  const int num_rows = J.num_rows;
  const int num_cols = J.num_cols;
  const int nnz      = static_cast<int>(J.values.size());

  if (num_rows < 0 || num_cols < 0) {
    return false;
  }
  if (static_cast<int>(J.rows.size()) != num_rows + 1) {
    return false;
  }
  if (static_cast<int>(J.cols.size()) != nnz) {
    return false;
  }
  if (J.rows.front() != 0 || J.rows.back() != nnz) {
    return false;
  }

  // CSR -> CSC transpose via counting sort (histogram, prefix-sum, scatter,
  // then shift offsets back).
  std::vector<int>    tRows(num_cols + 1, 0);
  std::vector<int>    tCols(nnz, 0);
  std::vector<double> tValues(nnz, 0);

  for (int idx = 0; idx < nnz; ++idx) {
    const int col = J.cols[idx];
    if (col < 0 || col >= num_cols) {
      return false;
    }
    tRows[col + 1] += 1;
  }

  for (int i = 1; i < static_cast<int>(tRows.size()); ++i) {
    tRows[i] += tRows[i - 1];
  }

  for (int r = 0; r < num_rows; ++r) {
    const int row_begin = J.rows[r];
    const int row_end   = J.rows[r + 1];
    if (row_begin < 0 || row_end < row_begin || row_end > nnz) {
      return false;
    }
    for (int idx = row_begin; idx < row_end; ++idx) {
      const int c             = J.cols[idx];
      const int transpose_idx = tRows[c];
      tCols[transpose_idx]    = r;
      tValues[transpose_idx]  = J.values[idx];
      ++tRows[c];
    }
  }

  for (int i = tRows.size() - 1; i > 0; --i) {
    tRows[i] = tRows[i - 1];
  }
  tRows[0] = 0;

  Jt.num_rows = num_cols;
  Jt.num_cols = num_rows;
  Jt.rows.swap(tRows);
  Jt.cols.swap(tCols);
  Jt.values.swap(tValues);
  return true;
}

inline bool create_hessian_from_crs_matrix(const ceres::CRSMatrix&    crsJ,
                                           const std::vector<double>& residuals,
                                           Eigen::MatrixXd&           H,
                                           Eigen::VectorXd&           JtR) {
  H.resize(0, 0);
  JtR.resize(0);

  if (crsJ.num_rows < 0 || crsJ.num_cols < 0) {
    return false;
  }
  if (static_cast<int>(residuals.size()) != crsJ.num_rows) {
    return false;
  }

  ceres::CRSMatrix crsJt;

  if (!transpose_crs_matrix(crsJ, crsJt)) {
    return false;
  }

  const int num_rows = crsJt.num_rows;

  H.resize(num_rows, num_rows);
  H.setZero();

  const std::vector<int>&    rows   = crsJt.rows;
  const std::vector<int>&    cols   = crsJt.cols;
  const std::vector<double>& values = crsJt.values;

  // H = Jt*J: each entry is a sparse dot-product of rows i,j of Jt via a
  // merge-join over ascending column indices; non-overlapping rows stay 0.
  for (int i = 0; i < num_rows; i++) {
    int i_colIdx     = rows[i];
    int i_colIdx_end = rows[i + 1];

    if (i_colIdx == i_colIdx_end)
      continue;

    for (int j = 0; j < num_rows; j++) {
      // Reset only the moving cursor; i_colIdx_end is loop-invariant.
      i_colIdx = rows[i];

      double val = 0;

      int j_colIdx     = rows[j];
      int j_colIdx_end = rows[j + 1];

      if (j_colIdx == j_colIdx_end)
        continue;

      while (i_colIdx < i_colIdx_end && j_colIdx < j_colIdx_end) {
        int i_col = cols[i_colIdx];
        int j_col = cols[j_colIdx];

        if (i_col < j_col) {
          ++i_colIdx;
        }
        else if (i_col > j_col) {
          ++j_colIdx;
        }
        else {
          val += (values[i_colIdx] * values[j_colIdx]);
          ++i_colIdx;
          ++j_colIdx;
        }
      }

      H(i, j) = val;
    }
  }

  JtR.resize(num_rows);
  JtR.setZero();

  for (int i = 0; i < num_rows; i++) {
    int i_colIdx     = rows[i];
    int i_colIdx_end = rows[i + 1];

    if (i_colIdx == i_colIdx_end)
      continue;

    double val = 0;

    for (int j = i_colIdx; j < i_colIdx_end; j++) {
      int col = cols[j];

      val += residuals[col] * values[j];
    }

    JtR(i) = val;
  }
  return true;
}
}  // namespace omni_slam::CeresUtil
