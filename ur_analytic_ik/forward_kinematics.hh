#include <Eigen/Dense>

using Matrix4x4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

inline Matrix4x4 dh_matrix(double theta_i, double d_i, double a_i, double alpha_i) {
  // TODO: figure out how to make this more legible
  Matrix4x4 m;
  m << cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i), sin(theta_i),
      cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i), 0, sin(alpha_i), cos(alpha_i),
      d_i, 0, 0, 0, 1;
  return m;
}

Matrix4x4 forward_kinematics(double theta1,
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

  const double alpha1 = M_PI_2;
  const double alpha4 = M_PI_2;
  const double alpha5 = -M_PI_2;

  Matrix4x4 T01 = dh_matrix(theta1, d1, 0.0, alpha1);
  Matrix4x4 T12 = dh_matrix(theta2, 0.0, a2, 0.0);
  Matrix4x4 T23 = dh_matrix(theta3, 0.0, a3, 0.0);
  Matrix4x4 T34 = dh_matrix(theta4, d4, 0.0, alpha4);
  Matrix4x4 T45 = dh_matrix(theta5, d5, 0.0, alpha5);
  Matrix4x4 T56 = dh_matrix(theta6, d6, 0.0, 0.0);

  return T01 * T12 * T23 * T34 * T45 * T56;
}

Matrix4x4 ur3e_forward_kinematics(
    double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
  const double d1 = 0.1519;
  const double d4 = 0.11235;
  const double d5 = 0.08535;
  const double d6 = 0.0819;
  const double a2 = -0.24365;
  const double a3 = -0.21325;

  return forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, d1, d4, d5, d6, a2, a3);
}

Matrix4x4 ur5e_forward_kinematics(
    double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
  // TODO avoid duplication of DH parameters between IK and FK file
  const double d1 = 0.1625;
  const double d4 = 0.1333;
  const double d5 = 0.0997;
  const double d6 = 0.0996;
  const double a2 = -0.425;
  const double a3 = -0.39225;

  return forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, d1, d4, d5, d6, a2, a3);
}
