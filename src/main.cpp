#include "ur_analytic_ik/inverse_kinematics.hh"
#include "ur_analytic_ik/utils.h"
#include <iostream>

int main()
{
  Matrix4x4 X_at_zero_truncated;
  X_at_zero_truncated << 
      1.0, 0.0, 0.0, -0.4569,
      0.0, 0.0, -1.0, -0.1943,
      0.0, 1.0, 0.0, 0.0666,
      0.0, 0.0, 0.0, 1.0;

  Matrix4x4 X_at_zero_more_precision;
  X_at_zero_more_precision << 
      1.0, 0.0, 0.0, -0.4569,
      0.0, 0.0, -1.0, -0.19425,
      0.0, 1.0, 0.0, 0.06655,
      0.0, 0.0, 0.0, 1.0;


  std::vector<Matrix1x6> q_solutions;
  q_solutions = ur3e::inverse_kinematics(X_at_zero_truncated);
  std::cout << "there are " << q_solutions.size() << " IK solutions" << std::endl;
  std::cout <<  vector_with_matrices_to_string(q_solutions) << std::endl;

  q_solutions = ur3e::inverse_kinematics(X_at_zero_more_precision);
  std::cout << "there are " << q_solutions.size() << " IK solutions" << std::endl;
  std::cout <<  vector_with_matrices_to_string(q_solutions) << std::endl;


  return 0;
}