#pragma once

#include "dh_parameters.hh"
#include "forward_kinematics.hh"
#include "utils.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

using Matrix4x4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix8x6 = Eigen::Matrix<double, 8, 6, Eigen::RowMajor>;
using Matrix1x6 = Eigen::Matrix<double, 1, 6, Eigen::RowMajor>;

tuple<double, double> calculate_theta1(double r13, double r23, double px, double py, double d4, double d6) {
  const double A1 = px - d6 * r13;
  const double B1 = d6 * r23 - py;

  // We use the letter H for helper variables that don't have a name in the paper.
  const double H1 = pow(A1, 2) + pow(B1, 2) - pow(d4, 2);
  const double H2 = atan2(sqrt(H1), d4);

  // Equation (24)
  const double theta1a = atan2(A1, B1) + H2;
  const double theta1b = atan2(A1, B1) - H2;

  return {theta1a, theta1b};
}

tuple<double, double> calculate_theta5(
    double c1, double s1, double r11, double r12, double r13, double r21, double r22, double r23) {

  // Equation (25) and (26)
  const double c5 = (s1 * r13) - (c1 * r23);
  const double s5 = sqrt(pow(s1 * r11 - c1 * r21, 2) + pow(s1 * r12 - c1 * r22, 2));

  // Equation (27)
  const double theta5a = atan2(s5, c5);
  const double theta5b = atan2(-s5, c5);

  return {theta5a, theta5b};
}

double calculate_theta6(double sign5, double c1, double s1, double r11, double r12, double r21, double r22) {
  double H1 = (c1 * r22) - (s1 * r12);
  double H2 = (s1 * r11) - (c1 * r21);

  // Snap M2 and M3 to zero if they are close to zero
  if (abs(H1) < 1e-12) {  // Threshold chosen arbitrarily
    H1 = 0.0;
  }
  if (abs(H2) < 1e-12) {
    H2 = 0.0;
  }

  // Equation (28)
  const double theta6 = atan2(sign5 * H1, sign5 * H2);
  return theta6;
}

tuple<double, double> calculate_theta3(double KS, double KC, double a2, double a3) {
  const double H1 = (pow(KS, 2) + pow(KC, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
  double H2 = 1.0 - pow(H1, 2);

  if (abs(H2) < 1e-12) {  // Threshold chosen arbitrarily
    H2 = 0.0;
  }

  // Note: Apparently, H2 can be negative, the sqrt() below then returns NaN.
  // When this happens, we simply keep going and put the NaN in the solutions matrix.
  // I believe this represents a scenario where there are less than 8 solutions.

  // Equation (58)
  const double theta3a = atan2(sqrt(H2), H1);
  const double theta3b = -theta3a;
  return {theta3a, theta3b};
}

double calculate_theta2(double KS, double KC, double c3, double s3, double a2, double a3) {
  // Equation (59)
  const double theta2 = atan2(KS, KC) - atan2(a3 * s3, (a3 * c3) + a2);
  return theta2;
}

std::vector<Matrix1x6> filter_solutions(const Matrix8x6 &solutions) {
  std::vector<Matrix1x6> valid_unique_solutions;

  for (int i = 0; i < solutions.rows(); i++) {
    const Matrix1x6 solution = solutions.row(i);

    // Checking if the solution is valid
    if (not solution.array().isFinite().all()) {
      continue;
    }

    // Checking if the solution is unique
    bool is_unique = true;
    for (const auto &valid_unique_solution : valid_unique_solutions) {
      if ((solution - valid_unique_solution).norm() < 1e-12) {
        is_unique = false;
        break;
      }
    }

    if (is_unique) {
      valid_unique_solutions.push_back(solution);
    }
  }

  return valid_unique_solutions;
}

std::vector<Matrix1x6> filter_solutions_with_forwards_kinematics(const std::vector<Matrix1x6> &solutions,
                                                                 const Matrix4x4 &desired_EEF_pose,
                                                                 double d1,
                                                                 double d4,
                                                                 double d5,
                                                                 double d6,
                                                                 double a2,
                                                                 double a3,
                                                                 double element_tolerance) {

  // This function is our last defense against incorrect solutions getting passed to users.
  // Ideally, we would never need this function, and would handle all edge cases in the calculations.
  std::vector<Matrix1x6> filtered_solutions;

  for (const auto &solution : solutions) {
    // Calculate forward kinematics for the given solution
    Matrix4x4 fk_pose = ur_forward_kinematics(
        solution(0), solution(1), solution(2), solution(3), solution(4), solution(5), d1, d4, d5, d6, a2, a3);

    // Element-wise comparison
    bool solution_correct = true;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        double error = std::abs(desired_EEF_pose(i, j) - fk_pose(i, j));
        if (error > element_tolerance) {
          solution_correct = false;
          break;  // No need to continue comparing if one element fails
        }
      }
      if (!solution_correct) {
        std::cout << "Warning: Detected and filtered incorrect IK solution: " << solution << std::endl;
        break;  // No need to iterate further if one row fails
      }
    }

    // Filter based on element-wise comparison
    if (solution_correct) {
      filtered_solutions.push_back(solution);
    }
  }

  return filtered_solutions;
}

// std::vector<Matrix1x6> filtered_solutions;

// for (const auto &solution : solutions) {
//   // Calculate forward kinematics for the given solution
//   Matrix4x4 fk_pose = ur_forward_kinematics(
//       solution(0), solution(1), solution(2), solution(3), solution(4), solution(5), d1, d4, d5, d6, a2, a3);

//   // Calculate pose error (you may want a different error metric)
//   Matrix4x4 error_pose = desired_EEF_pose.inverse() * fk_pose;
//   double error = error_pose.norm();  // Example: Frobenius norm
//   // double error = 0.0;

//   // Filter based on the error
//   std::cout << "Error: " << error << std::endl;
//   if (error <= tolerance) {
//     filtered_solutions.push_back(solution);
//   } else {
//     std::cout << "Solution filtered out: " << solution << " with error: " << error <<  std::endl;
//   }
// }

//   return filtered_solutions;
// }

std::vector<Matrix1x6> ur_inverse_kinematics(
    const Matrix4x4 &desired_EEF_pose, double d1, double d4, double d5, double d6, double a2, double a3) {

  // Unpacking the matrix
  const Matrix4x4 &M = desired_EEF_pose;
  const double r11 = M(0, 0), r12 = M(0, 1), r13 = M(0, 2);
  const double r21 = M(1, 0), r22 = M(1, 1), r23 = M(1, 2);
  const double r31 = M(2, 0), r32 = M(2, 1), r33 = M(2, 2);
  const double px = M(0, 3), py = M(1, 3), pz = M(2, 3);

  // Solutions matrix, each row is a solution, each column is a joint angle, starting from theta1
  // Matrix8x6 solutions;
  Matrix8x6 solutions = Matrix8x6::Zero();

  // Calculating the two solutions for theta1
  const auto [theta1a, theta1b] = calculate_theta1(r13, r23, px, py, d4, d6);
  solutions.col(0).topRows(4).array() = theta1a;
  solutions.col(0).bottomRows(4).array() = theta1b;

  // Calculating the four solutions for theta5 and theta6
  for (int i : {0, 4}) {
    const double theta1 = solutions(i, 0);
    const double c1 = cos(theta1);
    const double s1 = sin(theta1);

    const auto [theta5a, theta5b] = calculate_theta5(c1, s1, r11, r12, r13, r21, r22, r23);

    const double s5a = sin(theta5a);
    const double s5b = sin(theta5b);
    double sign5a = s5a / abs(s5a);
    double sign5b = s5b / abs(s5b);

    // s5a and s5b can be both zero, then lets put +0 always in 'a' and negative zero in 'b'
    if (abs(s5a - 0.0) <= 1e-12) {
      sign5a = 1;
      sign5b = -1;
    }
    const double theta6a = calculate_theta6(sign5a, c1, s1, r11, r12, r21, r22);
    const double theta6b = calculate_theta6(sign5b, c1, s1, r11, r12, r21, r22);

    solutions.col(4).middleRows(i, 2).array() = theta5a;
    solutions.col(5).middleRows(i, 2).array() = theta6a;
    solutions.col(4).middleRows(i + 2, 2).array() = theta5b;
    solutions.col(5).middleRows(i + 2, 2).array() = theta6b;
  }

  // Calculating the eight solutions for theta3, theta2 and theta4
  for (int i : {0, 2, 4, 6}) {
    const double theta1 = solutions(i, 0);
    const double theta5 = solutions(i, 4);
    const double theta6 = solutions(i, 5);

    // Recalculating these sines and cosines is not stictly necessary, as we already used them in the previous loop.
    // I'm not sure if it's worth to scarifice readability for the potential performance gain.
    const double c1 = cos(theta1);
    const double s1 = sin(theta1);
    const double c5 = cos(theta5);
    const double s5 = sin(theta5);
    const double c6 = cos(theta6);
    const double s6 = sin(theta6);

    // Equation (29)
    const double A234 = (c1 * r11) + (s1 * r21);
    const double H1 = (c5 * c6 * r31) - (s6 * A234);
    const double H2 = (c5 * c6 * A234) + (s6 * r31);

    // Equation (30)
    const double theta234 = atan2(H1, H2);
    const double c234 = cos(theta234);
    const double s234 = sin(theta234);

    // Calculating theta3
    // Equation (52) and (55)
    const double KC = (c1 * px) + (s1 * py) - (s234 * d5) + (c234 * s5 * d6);
    const double KS = pz - d1 + (c234 * d5) + (s234 * s5 * d6);

    const auto [theta3a, theta3b] = calculate_theta3(KS, KC, a2, a3);
    // Calculating theta2
    const double c3a = cos(theta3a);
    const double s3a = sin(theta3a);
    const double theta2a = calculate_theta2(KS, KC, c3a, s3a, a2, a3);

    const double c3b = cos(theta3b);
    const double s3b = sin(theta3b);
    const double theta2b = calculate_theta2(KS, KC, c3b, s3b, a2, a3);

    // Calculating theta4
    const double theta4a = theta234 - theta2a - theta3a;
    const double theta4b = theta234 - theta2b - theta3b;

    solutions.col(1).row(i).array() = theta2a;
    solutions.col(2).row(i).array() = theta3a;
    solutions.col(3).row(i).array() = theta4a;

    solutions.col(1).row(i + 1).array() = theta2b;
    solutions.col(2).row(i + 1).array() = theta3b;
    solutions.col(3).row(i + 1).array() = theta4b;
  }

  // At this points, the angles lie in [-2pi, 2pi]
  // However, for standardization we map them to [-pi, pi]
  solutions = solutions.unaryExpr([](const double x) {
    const double y = fmod(x, 2.0 * M_PI);  // Not 100% sure if this is necessary, but the tests failed without it.
    if (y > M_PI) {
      return y - 2.0 * M_PI;
    } else if (y < -M_PI) {
      return y + 2.0 * M_PI;
    }
    return y;
  });
  std::vector<Matrix1x6> valid_unique_solutions = filter_solutions(solutions);
  std::vector<Matrix1x6> valid_unique_solutions_with_fk = filter_solutions_with_forwards_kinematics(
      valid_unique_solutions, desired_EEF_pose, d1, d4, d5, d6, a2, a3, 1e-9);

  return valid_unique_solutions_with_fk;
}

std::vector<Matrix1x6> closest_solution(std::vector<Matrix1x6> &solutions, const Matrix1x6 &joint_angles) {
  if (solutions.empty()) {
    return solutions;
  }

  // Consider that the user may pass current joints in the range [-2pi, 2pi]
  // However, the raw solutions are in the range [-pi, pi]
  // For the convenience of the user, we return the closest solution in the range [-2pi, 2pi]
  // This requires some care in the distance calculation.
  // We do this in two stage: first we map the joint_angles to the range [-pi, pi] and find the closest solution.
  // Then we construct the alternative solution by adding or subtracting 2pi to the closest solution.
  // Finally, for each angle we pick the one that is closer to the original joint_angles.

  Matrix1x6 joint_angles_mapped = joint_angles.unaryExpr([](const double x) {
    const double y = fmod(x, 2.0 * M_PI);
    if (y > M_PI) {
      return y - 2.0 * M_PI;
    } else if (y < -M_PI) {
      return y + 2.0 * M_PI;
    }
    return y;
  });

  Matrix1x6 closest_solution = solutions[0];
  double closest_distance = (joint_angles_mapped - closest_solution).norm();

  for (const auto &solution : solutions) {
    // const double distance = (joint_angles_mapped - solution).norm();
    double distance = 0.0;
    for (int i = 0; i < 6; i++) {
      double difference = std::abs(joint_angles_mapped(i) - solution(i));
      if (difference > M_PI) {
        difference = 2.0 * M_PI - difference;
      }
      distance += difference;
    }

    if (distance < closest_distance) {
      closest_solution = solution;
      closest_distance = distance;
    }
  }

  // Alternative solution is either +2pi or -2pi depending on the sign of the closest solution.
  Matrix1x6 alternative_solution = closest_solution.unaryExpr([](const double x) {
    if (x < 0) {
      return x + 2.0 * M_PI;
    } else {
      return x - 2.0 * M_PI;
    }
  });

  // Final solutions contains the closest of closest_solution and alternative_solution.
  Matrix1x6 final_solution;
  for (int i = 0; i < 6; i++) {
    double difference_to_closest = std::abs(closest_solution(i) - joint_angles(i));
    double difference_to_alternative = std::abs(alternative_solution(i) - joint_angles(i));
    if (difference_to_closest < difference_to_alternative) {
      final_solution(i) = closest_solution(i);
    } else {
      final_solution(i) = alternative_solution(i);
    }
  }

  return {final_solution};
}

// Robot specifics functions below here.

namespace ur3e {
std::vector<Matrix1x6> inverse_kinematics(const Matrix4x4 &desired_EEF_pose) {
  return ur_inverse_kinematics(desired_EEF_pose, d1, d4, d5, d6, a2, a3);
}

std::vector<Matrix1x6> inverse_kinematics_with_tcp(const Matrix4x4 &desired_EEF_pose, const Matrix4x4 &tcp_transform) {
  return inverse_kinematics(desired_EEF_pose * tcp_transform.inverse());
}

std::vector<Matrix1x6> inverse_kinematics_closest(const Matrix4x4 &desired_EEF_pose,
                                                  double theta1,
                                                  double theta2,
                                                  double theta3,
                                                  double theta4,
                                                  double theta5,
                                                  double theta6) {
  std::vector<Matrix1x6> solutions = inverse_kinematics(desired_EEF_pose);
  Matrix1x6 joint_angles;
  joint_angles << theta1, theta2, theta3, theta4, theta5, theta6;
  return closest_solution(solutions, joint_angles);
}

std::vector<Matrix1x6> inverse_kinematics_closest_with_tcp(const Matrix4x4 &desired_EEF_pose,
                                                           const Matrix4x4 &tcp_transform,
                                                           double theta1,
                                                           double theta2,
                                                           double theta3,
                                                           double theta4,
                                                           double theta5,
                                                           double theta6) {
  return inverse_kinematics_closest(
      desired_EEF_pose * tcp_transform.inverse(), theta1, theta2, theta3, theta4, theta5, theta6);
}
}  // namespace ur3e

namespace ur5e {
std::vector<Matrix1x6> inverse_kinematics(const Matrix4x4 &desired_EEF_pose) {
  return ur_inverse_kinematics(desired_EEF_pose, d1, d4, d5, d6, a2, a3);
}

std::vector<Matrix1x6> inverse_kinematics_with_tcp(const Matrix4x4 &desired_EEF_pose, const Matrix4x4 &tcp_transform) {
  return inverse_kinematics(desired_EEF_pose * tcp_transform.inverse());
}

std::vector<Matrix1x6> inverse_kinematics_closest(const Matrix4x4 &desired_EEF_pose,
                                                  double theta1,
                                                  double theta2,
                                                  double theta3,
                                                  double theta4,
                                                  double theta5,
                                                  double theta6) {
  std::vector<Matrix1x6> solutions = inverse_kinematics(desired_EEF_pose);
  Matrix1x6 joint_angles;
  joint_angles << theta1, theta2, theta3, theta4, theta5, theta6;
  return closest_solution(solutions, joint_angles);
}

std::vector<Matrix1x6> inverse_kinematics_closest_with_tcp(const Matrix4x4 &desired_EEF_pose,
                                                           const Matrix4x4 &tcp_transform,
                                                           double theta1,
                                                           double theta2,
                                                           double theta3,
                                                           double theta4,
                                                           double theta5,
                                                           double theta6) {
  return inverse_kinematics_closest(
      desired_EEF_pose * tcp_transform.inverse(), theta1, theta2, theta3, theta4, theta5, theta6);
}

}  // namespace ur5e

namespace ur10e {
std::vector<Matrix1x6> inverse_kinematics(const Matrix4x4 &desired_EEF_pose) {
  return ur_inverse_kinematics(desired_EEF_pose, d1, d4, d5, d6, a2, a3);
}

std::vector<Matrix1x6> inverse_kinematics_with_tcp(const Matrix4x4 &desired_EEF_pose, const Matrix4x4 &tcp_transform) {
  return inverse_kinematics(desired_EEF_pose * tcp_transform.inverse());
}

std::vector<Matrix1x6> inverse_kinematics_closest(const Matrix4x4 &desired_EEF_pose,
                                                  double theta1,
                                                  double theta2,
                                                  double theta3,
                                                  double theta4,
                                                  double theta5,
                                                  double theta6) {
  std::vector<Matrix1x6> solutions = inverse_kinematics(desired_EEF_pose);

  Matrix1x6 joint_angles;
  joint_angles << theta1, theta2, theta3, theta4, theta5, theta6;
  return closest_solution(solutions, joint_angles);
}

std::vector<Matrix1x6> inverse_kinematics_closest_with_tcp(const Matrix4x4 &desired_EEF_pose,
                                                           const Matrix4x4 &tcp_transform,
                                                           double theta1,
                                                           double theta2,
                                                           double theta3,
                                                           double theta4,
                                                           double theta5,
                                                           double theta6) {
  return inverse_kinematics_closest(
      desired_EEF_pose * tcp_transform.inverse(), theta1, theta2, theta3, theta4, theta5, theta6);
}

}  // namespace ur10e
