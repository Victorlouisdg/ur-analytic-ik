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

template <typename T>
struct UR_FK_Logic
{
  static Matrix4x4 forward_kinematics(
      double t1, double t2, double t3, double t4, double t5, double t6)
  {
    return ur_forward_kinematics(t1, t2, t3, t4, t5, t6, T::d1, T::d4, T::d5, T::d6, T::a2, T::a3);
  }

  static Matrix4x4 forward_kinematics_with_tcp(
      double t1, double t2, double t3, double t4, double t5, double t6, const Matrix4x4 &tcp)
  {
    return forward_kinematics(t1, t2, t3, t4, t5, t6) * tcp;
  }
};

namespace ur3
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur3e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur5
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur5e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur7e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur8long
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur10
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur10e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur12e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur15
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur16e
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur18
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur20
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}

namespace ur30
{
  using FK = UR_FK_Logic<Args>;

  constexpr auto forward_kinematics = FK::forward_kinematics;
  constexpr auto forward_kinematics_with_tcp = FK::forward_kinematics_with_tcp;
}
