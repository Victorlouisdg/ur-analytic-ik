#include <Eigen/Core>
#include <iostream>
#include <math.h>

using namespace std;

using Matrix4x4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix8x6 = Eigen::Matrix<double, 8, 6, Eigen::RowMajor>;

tuple<double, double> calculate_theta1(double r13, double r23, double px, double py, double d4, double d6) {
  const double A1 = px - d6 * r13;
  const double B1 = d6 * r23 - py;

  // I use the letter H for helper variables that don't have a name in the paper.
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

  // Note: Appartenently, H2 can be negative, the sqrt() below then returns NaN.
  // When this happens, we simply keep going and put the NaN in the solutions matrix.
  // I believe ehis represents a scenario where there are <8 solutions.

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

Matrix8x6 ur5e_inverse_kinematics(Matrix4x4 desired_EEF_pose) {
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
    const double sign5a = s5a / abs(s5a);
    const double sign5b = s5b / abs(s5b);

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
  // However, we want [0 , 2pi[ take the modulo 2pi and handle the negative values
  solutions = solutions.unaryExpr([](const double x) {
    const double y = fmod(x, 2.0 * M_PI);
    if (y > 0.0) {
      return y;
    } else {
      return y + 2.0 * M_PI;
    }
    // else return fmod(x + 4.0 * M_PI, 2.0 * M_PI);
  });

  return solutions;
}

// const auto [theta2a, theta2b] = calculate_theta2(c234, s234, c1, s1, s5, px, py, pz, d1, d5, d6, a2, a3);
// solutions.col(1).row(i).array() = theta2a;
// solutions.col(1).row(i + 1).array() = theta2b;

// tuple<double, double> calculate_theta2_equation34(double c234,
//                                        double s234,
//                                        double c1,
//                                        double s1,
//                                        double s5,
//                                        double px,
//                                        double py,
//                                        double pz,
//                                        double d1,
//                                        double d5,
//                                        double d6,
//                                        double a2,
//                                        double a3) {
//   // Equation (31), (32)
//   const double A2 = 2 * a2 * (d1 - pz - (d5 * c234) - (d6 * s5 * s234));
//   const double B2 = 2 * a2 * ((d5 * s234) - (d6 * s5 * c234) - (c1 * px) - (s1 * py));

//   const double H1 = pow(a3, 2) - pow(a2, 2) - pow(d5, 2);
//   const double H2 = (pz - d1) * ((2 * d5 * c234) + (2 * d6 * s5 * s234) + pz - d1);
//   const double H3 = (c1 * px) + (s1 * py);
//   const double H4 = (2 * d5 * s234) - (2 * d6 * s5 * c234) - (c1 * px) - (s1 * py);
//   const double H5 = pow(d6, 2) * pow(s5, 2);

//   // Equation (33)
//   const double C2 = H1 - H2 + (H3 * H4) - H5;

//   double H6 = pow(A2, 2) + pow(B2, 2) - pow(C2, 2);

//   if (abs(H6) < 1e-12) {  // Threshold chosen arbitrarily
//     H6 = 0.0;
//   }

//   const double H7 = atan2(sqrt(H6), C2);

//   // Equation (34)
//   const double theta2a = atan2(A2, B2) + H7;
//   const double theta2b = atan2(A2, B2) - H7;
//   return {theta2a, theta2b};
// }
