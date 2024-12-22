#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

using namespace nb::literals;
using namespace std;

void define_forward_kinematics(nb::module_ &robot_module,
                               std::function<Matrix4x4(Vector6d,Matrix4x4)> fk_function) {
  robot_module.def("forward_kinematics",
                   [=](Vector6d joint_angles, Matrix4x4 tcp_in_flange_pose = Matrix4x4::Identity()) {
                     // Call the FK function
                     Matrix4x4 rowMajorMatrix = fk_function(joint_angles, tcp_in_flange_pose);
                     return rowMajorMatrix;
                   }, "joint_angles"_a, "tcp_pose"_a = Matrix4x4::Identity(),
                   "Calculates the forward kinematics of the robot, providing the TCP pose in the robot's base frame for a given joint configuration"
                   //TODO(tlpss): figure out how to use multi-line docstrings 

                  //  "Calculates the forward kinematics of the robot, providing the TCP pose in the robot's base frame for a given joint configuration
                  //  Also takes in the offset (pose) of the TCP w.r.t. the flange frame, as the FK function only knows the kinematics of the robot, not the tool.

                  //   :param joint_angles: A 6-element vector containing the joint angles in radians.
                  //   :param tcp_pose: A 4x4 matrix representing the pose of the TCP in the robot tool frame. Defaults to identity, in which case the TCP is assumed to be at the flange.

                  //   :return: A 4x4 matrix representing the pose of the TCP in the robot base frame.
                  //  "
                   );
}


void define_inverse_kinematics(nb::module_ &robot_module,
                               std::function<std::vector<Matrix1x6>(Matrix4x4)> ik_function) {
  robot_module.def("inverse_kinematics", [=](Matrix4x4 tensor) {
    // Call the IK function
    std::vector<Matrix1x6> solutions = ik_function(tensor);
    return solutions;
  });
}

void define_inverse_kinematics_closest(
    nb::module_ &robot_module,
    std::function<std::vector<Matrix1x6>(Matrix4x4, double, double, double, double, double, double)>
        ik_closest_function) {
  robot_module.def(
      "inverse_kinematics_closest",
      [=](Matrix4x4 tensor, double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
        // Call the IK function
        std::vector<Matrix1x6> solutions = ik_closest_function(tensor, theta1, theta2, theta3, theta4, theta5, theta6);

        return solutions;
      });
}

void define_inverse_kinematics_with_tcp(
    nb::module_ &robot_module, std::function<std::vector<Matrix1x6>(Matrix4x4, Matrix4x4)> ik_with_tcp_function) {
  robot_module.def("inverse_kinematics_with_tcp", [=](Matrix4x4 tensor, Matrix4x4 tcp_transform) {
    // Call the IK function
    std::vector<Matrix1x6> solutions = ik_with_tcp_function(tensor, tcp_transform);

    return solutions;
  });
}

void define_inverse_kinematics_closest_with_tcp(
    nb::module_ &robot_module,
    std::function<std::vector<Matrix1x6>(Matrix4x4, Matrix4x4, double, double, double, double, double, double)>
        ik_closest_with_tcp_function) {
  robot_module.def("inverse_kinematics_closest_with_tcp",
                   [=](Matrix4x4 tensor,
                       Matrix4x4 tcp_transform,
                       double theta1,
                       double theta2,
                       double theta3,
                       double theta4,
                       double theta5,
                       double theta6) {
                     // Call the IK function
                     std::vector<Matrix1x6> solutions = ik_closest_with_tcp_function(
                         tensor, tcp_transform, theta1, theta2, theta3, theta4, theta5, theta6);
                     return solutions;
                   });
}

NB_MODULE(ur_analytic_ik_ext, m) {
  nb::module_ m_ur3e = m.def_submodule("ur3e", "UR3e module");
  define_forward_kinematics(m_ur3e, ur3e::forward_kinematics);
  define_inverse_kinematics(m_ur3e, ur3e::inverse_kinematics);
  define_inverse_kinematics_closest(m_ur3e, ur3e::inverse_kinematics_closest);
  define_inverse_kinematics_with_tcp(m_ur3e, ur3e::inverse_kinematics_with_tcp);
  define_inverse_kinematics_closest_with_tcp(m_ur3e, ur3e::inverse_kinematics_closest_with_tcp);

  nb::module_ m_ur5e = m.def_submodule("ur5e", "UR5e module");
  define_forward_kinematics(m_ur5e, ur5e::forward_kinematics);
  define_inverse_kinematics(m_ur5e, ur5e::inverse_kinematics);
  define_inverse_kinematics_closest(m_ur5e, ur5e::inverse_kinematics_closest);
  define_inverse_kinematics_with_tcp(m_ur5e, ur5e::inverse_kinematics_with_tcp);
  define_inverse_kinematics_closest_with_tcp(m_ur5e, ur5e::inverse_kinematics_closest_with_tcp);

  nb::module_ m_ur10e = m.def_submodule("ur10e", "UR10e module");
  define_forward_kinematics(m_ur10e, ur10e::forward_kinematics);
  define_inverse_kinematics(m_ur10e, ur10e::inverse_kinematics);
  define_inverse_kinematics_closest(m_ur10e, ur10e::inverse_kinematics_closest);
  define_inverse_kinematics_with_tcp(m_ur10e, ur10e::inverse_kinematics_with_tcp);
  define_inverse_kinematics_closest_with_tcp(m_ur10e, ur10e::inverse_kinematics_closest_with_tcp);

  // This function is mostly still here to understand nanobind.
  // Once we properly handle tensors without copying etc, we can remove this.
  m.def("inspect", [](nb::ndarray<> tensor) {
    printf("Tensor data pointer : %p\n", tensor.data());
    printf("Tensor dimension : %zu\n", tensor.ndim());
    for (size_t i = 0; i < tensor.ndim(); ++i) {
      printf("Tensor dimension [%zu] : %zu\n", i, tensor.shape(i));
      printf("Tensor stride    [%zu] : %zd\n", i, tensor.stride(i));
    }
    printf("Device ID = %u (cpu=%i, cuda=%i)\n",
           tensor.device_id(),
           int(tensor.device_type() == nb::device::cpu::value),
           int(tensor.device_type() == nb::device::cuda::value));
    printf("Tensor dtype check: int16=%i, uint32=%i, float32=%i, double=%i\n",
           tensor.dtype() == nb::dtype<int16_t>(),
           tensor.dtype() == nb::dtype<uint32_t>(),
           tensor.dtype() == nb::dtype<float>(),
           tensor.dtype() == nb::dtype<double>());
  });
}
