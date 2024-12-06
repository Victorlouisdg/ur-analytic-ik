
#ifndef MAIN_UTILS_H
#define MAIN_UTILS_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <iomanip>
#include <iostream>
#include <vector>

//static std::string matrixXd_to_string(const Eigen::MatrixXd &matrix) {
//  std::string matrixString = "[";
//
//  for (int i = 0; i < matrix.rows(); ++i) {
//    for (int j = 0; j < matrix.cols(); ++j) {
//      matrixString += std::to_string(matrix(i, j)) + ",\t";
//    }
//    matrixString += "],\n";  // Add newline after each row
//  }
//  return matrixString;
//}

static std::string matrixXd_to_string(const Eigen::MatrixXd &matrix) {
  std::ostringstream matrixStringStream;

  matrixStringStream << std::fixed; // Set the floating-point output to fixed notation

  // Set the desired precision
  int precision = 10; // Change this value to your desired precision
  matrixStringStream << std::setprecision(precision);

  matrixStringStream << "[";

  for (int i = 0; i < matrix.rows(); ++i) {
//    matrixStringStream << "[";
    for (int j = 0; j < matrix.cols(); ++j) {
      matrixStringStream << matrix(i, j);
      if (j < matrix.cols() - 1) {
        matrixStringStream << ",\t";
      }
    }
//    matrixStringStream << "]\n";
    if (i < matrix.rows() - 1) {
      matrixStringStream << ",\n";
    }
  }

  matrixStringStream << "]";

  return matrixStringStream.str();
}

template <typename MatrixType>
static std::string vector_with_matrices_to_string(const std::vector<MatrixType> q_solutions) {
  std::string out_string;

  for (const auto &q : q_solutions) {
    out_string += matrixXd_to_string(q) + "\n";
  }

  return out_string;
}
#endif  // MAIN_UTILS_H
