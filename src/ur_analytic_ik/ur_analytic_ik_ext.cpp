#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

using namespace nb::literals;
using namespace std;

// TODO document the how/why of these define functions
void define_forward_kinematics(nb::module_ &robot_module,
                               std::function<Matrix4x4(double, double, double, double, double, double)> fk_function) {
  robot_module.def("forward_kinematics",
                   [=](double theta1, double theta2, double theta3, double theta4, double theta5, double theta6) {
                     // Call the FK function
                     Matrix4x4 rowMajorMatrix = fk_function(theta1, theta2, theta3, theta4, theta5, theta6);
                     return rowMajorMatrix;
                   });
}

void define_forward_kinematics_with_tcp(
    nb::module_ &robot_module,
    std::function<Matrix4x4(double, double, double, double, double, double, Matrix4x4)> fk_with_tcp_function) {
  robot_module.def("forward_kinematics_with_tcp",
                   [=](double theta1,
                       double theta2,
                       double theta3,
                       double theta4,
                       double theta5,
                       double theta6,
                       Matrix4x4 tcp_transform) {
                     // Call the FK function
                     Matrix4x4 rowMajorMatrix = fk_with_tcp_function(
                         theta1, theta2, theta3, theta4, theta5, theta6, tcp_transform);

                     return rowMajorMatrix;
                   });
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
  define_forward_kinematics_with_tcp(m_ur3e, ur3e::forward_kinematics_with_tcp);
  define_inverse_kinematics(m_ur3e, ur3e::inverse_kinematics);
  define_inverse_kinematics_closest(m_ur3e, ur3e::inverse_kinematics_closest);
  define_inverse_kinematics_with_tcp(m_ur3e, ur3e::inverse_kinematics_with_tcp);
  define_inverse_kinematics_closest_with_tcp(m_ur3e, ur3e::inverse_kinematics_closest_with_tcp);

  nb::module_ m_ur5e = m.def_submodule("ur5e", "UR5e module");
  define_forward_kinematics(m_ur5e, ur5e::forward_kinematics);
  define_forward_kinematics_with_tcp(m_ur5e, ur5e::forward_kinematics_with_tcp);
  define_inverse_kinematics(m_ur5e, ur5e::inverse_kinematics);
  define_inverse_kinematics_closest(m_ur5e, ur5e::inverse_kinematics_closest);
  define_inverse_kinematics_with_tcp(m_ur5e, ur5e::inverse_kinematics_with_tcp);
  define_inverse_kinematics_closest_with_tcp(m_ur5e, ur5e::inverse_kinematics_closest_with_tcp);

  nb::module_ m_ur10e = m.def_submodule("ur10e", "UR10e module");
  define_forward_kinematics(m_ur10e, ur10e::forward_kinematics);
  define_forward_kinematics_with_tcp(m_ur10e, ur10e::forward_kinematics_with_tcp);
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
