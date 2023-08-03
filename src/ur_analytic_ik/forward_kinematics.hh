#pragma once
#include "dh_parameters.hh"
#include <Eigen/Dense>

using Matrix4x4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Matrix4x4 dh_matrix(double theta_i, double d_i, double a_i, double alpha_i) {
  // TODO: figure out how to make this more legible
  Matrix4x4 m;
  m << cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i), sin(theta_i),
      cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i), 0, sin(alpha_i), cos(alpha_i),
      d_i, 0, 0, 0, 1;
  return m;
}

Matrix4x4 ur_forward_kinematics(double theta1,
                                double theta2,
                                double theta3,
                                double theta4,
                                double theta5,
                                double theta6,
                                double d1,
                                double d4,
                                double d5,
                                double d6,
                                double a2,
                                double a3) {
  Matrix4x4 T01 = dh_matrix(theta1, d1, 0.0, ur::alpha1);
  Matrix4x4 T12 = dh_matrix(theta2, 0.0, a2, 0.0);
  Matrix4x4 T23 = dh_matrix(theta3, 0.0, a3, 0.0);
  Matrix4x4 T34 = dh_matrix(theta4, d4, 0.0, ur::alpha4);
  Matrix4x4 T45 = dh_matrix(theta5, d5, 0.0, ur::alpha5);
  Matrix4x4 T56 = dh_matrix(theta6, d6, 0.0, 0.0);

  return T01 * T12 * T23 * T34 * T45 * T56;
}

namespace ur3e {
Matrix4x4 forward_kinematics(
    double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
  return ur_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, d1, d4, d5, d6, a2, a3);
}

Matrix4x4 forward_kinematics_with_tcp(double theta1,
                                      double theta2,
                                      double theta3,
                                      double theta4,
                                      double theta5,
                                      double theta6,
                                      const Matrix4x4 &tcp_transform) {
  return forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6) * tcp_transform;
}

}  // namespace ur3e

namespace ur5e {
Matrix4x4 forward_kinematics(
    double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
  return ur_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, d1, d4, d5, d6, a2, a3);
}

Matrix4x4 forward_kinematics_with_tcp(double theta1,
                                      double theta2,
                                      double theta3,
                                      double theta4,
                                      double theta5,
                                      double theta6,
                                      const Matrix4x4 &tcp_transform) {
  return forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6) * tcp_transform;
}

}  // namespace ur5e

namespace ur10e {
Matrix4x4 forward_kinematics(
    double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
  return ur_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, d1, d4, d5, d6, a2, a3);
}

Matrix4x4 forward_kinematics_with_tcp(double theta1,
                                      double theta2,
                                      double theta3,
                                      double theta4,
                                      double theta5,
                                      double theta6,
                                      const Matrix4x4 &tcp_transform) {
  return forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6) * tcp_transform;
}

}  // namespace ur10e
