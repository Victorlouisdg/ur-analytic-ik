#include <Eigen/Core>
#include <iostream>

using Matrix4x4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix8x6 = Eigen::Matrix<double, 8, 6, Eigen::RowMajor>;

Matrix8x6 ur5e_inverse_kinematics(const Matrix4x4 &desired_EEF_pose) {
  const Matrix4x4 &M = desired_EEF_pose;

  // Unpacking the matrix
  const double r11 = M(0, 0), r12 = M(0, 1), r13 = M(0, 2);
  const double r21 = M(1, 0), r22 = M(1, 1), r23 = M(1, 2);
  const double r31 = M(2, 0), r32 = M(2, 1), r33 = M(2, 2);
  const double px = M(0, 3), py = M(1, 3), pz = M(2, 3);

  // UR5e DH parameters
  const double d1 = 0.1625;
  const double d4 = 0.1333;
  const double d5 = 0.0997;
  const double d6 = 0.0996;
  const double a2 = -0.425;
  const double a3 = -0.39225;
  const double alpha1 = M_PI_2;
  const double alpha4 = M_PI_2;
  const double alpha5 = -M_PI_2;

  // Calculating theta1
  const double A1 = px - d6 * r13;
  const double B1 = d6 * r23 - py;

  // I use the letter M for helper variables that don't have a name in the
  // paper.
  // const double M1 = d4 * d4 - A1 * A1 - B1 * B1;
  const double M1 = pow(A1, 2) + pow(B1, 2) - pow(d4, 2);

  // TODO think about how to keep track of the two solutions for theta1.
  // Equation (24)
  const double theta1 = atan2(A1, B1) + atan2(sqrt(M1), d4);

  // Calculating theta5
  const double c1 = cos(theta1);
  const double s1 = sin(theta1);

  // Equation (25) and (26)
  const double c5 = (s1 * r13) - (c1 * r23);
  const double s5 = sqrt(pow(s1 * r11 - c1 * r21, 2) + pow(s1 * r12 - c1 * r22, 2));

  // Equation (27)
  const double theta5 = atan2(s5, c5);

  // Calculating theta6
  const double sign5 = s5 / abs(s5);
  double M2 = (c1 * r22) - (s1 * r12);

  double M3 = (s1 * r11) - (c1 * r21);

  // Snap M2 and M3 to zero if they are close to zero
  if (abs(M2) < 1e-12) {  // Threshold chosen arbitrarily
    M2 = 0.0;
  }
  if (abs(M3) < 1e-12) {
    M3 = 0.0;
  }

  // Equation (28)
  const double theta6 = atan2(sign5 * M2, sign5 * M3);
  // const double theta6 = atan2(M2 / sign5, M3 / sign5);

  // Calculating theta2
  const double c6 = cos(theta6);
  const double s6 = sin(theta6);

  // Equation (29)
  const double A234 = (c1 * r11) + (s1 * r21);
  const double M4 = (c5 * c6 * r31) - (s6 * A234);
  const double M5 = (c5 * c6 * A234) + (s6 * r31);

  // Equation (30)
  const double theta234 = atan2(M4, M5);

  const double c234 = cos(theta234);
  const double s234 = sin(theta234);

  // Equation (31), (32)
  const double A2 = 2 * a2 * (d1 - pz - (d5 * c234) - (d6 * s5 * s234));
  const double B2 = 2 * a2 * ((d5 * s234) - (d6 * s5 * c234) - (c1 * px) - (s1 * py));

  const double M6 = pow(a3, 2) - pow(a2, 2) - pow(d5, 2);
  const double M7 = (pz - d1) * ((2 * d5 * c234) + (2 * d6 * s5 * s234) + pz - d1);
  const double M8 = (c1 * px) + (s1 * py);
  const double M9 = (2 * d5 * s234) - (2 * d6 * s5 * c234) - (c1 * px) - (s1 * py);
  const double M10 = pow(d6, 2) * pow(s5, 2);

  // Equation (33)
  const double C2 = M6 - M7 + (M8 * M9) - M10;

  double M11 = pow(A2, 2) + pow(B2, 2) - pow(C2, 2);

  if (abs(M11) < 1e-12) {  // Threshold chosen arbitrarily
    M11 = 0.0;
  }

  // Equation (34)
  const double theta2 = atan2(A2, B2) + atan2(sqrt(M11), C2);

  // Calculating theta3
  // Equation (52) and (55)
  const double KC = (c1 * px) + (s1 * py) - (s234 * d5) + (c234 * s5 * d6);
  const double KS = pz - d1 + (c234 * d5) + (s234 * s5 * d6);

  const double M12 = (pow(KS, 2) + pow(KC, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
  double M13 = 1.0 - pow(M12, 2);
  if (abs(M13) < 1e-12) {  // Threshold chosen arbitrarily
    M13 = 0.0;
  }

  // Equation (58)
  const double theta3 = atan2(sqrt(M13), M12);

  // Calculating theta4
  const double theta4 = theta234 - theta2 - theta3;

  Matrix8x6 solutions;
  solutions.array().col(0) = theta1;
  solutions.array().col(1) = theta2;
  solutions.array().col(2) = theta3;
  solutions.array().col(3) = theta4;
  solutions.array().col(4) = theta5;
  solutions.array().col(5) = theta6;

  return solutions;
}
