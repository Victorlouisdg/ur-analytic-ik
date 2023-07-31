#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/ndarray.h>

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

                     // Copy returned Matrix to array of doubles
                     size_t shape[2] = {4, 4};
                     double *double_array = new double[16];
                     memcpy(double_array, rowMajorMatrix.data(), 16 * sizeof(double));
                     nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });

                     return nb::ndarray<nb::numpy, double, nb::shape<4, 4>>(double_array, 2, shape, deleter);
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
                       nb::ndarray<> tcp_transform) {
                     // Call the FK function
                     Matrix4x4 tcp_transform_eigen;
                     memcpy(tcp_transform_eigen.data(), tcp_transform.data(), 16 * sizeof(double));

                     Matrix4x4 rowMajorMatrix = fk_with_tcp_function(
                         theta1, theta2, theta3, theta4, theta5, theta6, tcp_transform_eigen);

                     // Copy returned Matrix to array of doubles
                     size_t shape[2] = {4, 4};
                     double *double_array = new double[16];
                     memcpy(double_array, rowMajorMatrix.data(), 16 * sizeof(double));
                     nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });

                     return nb::ndarray<nb::numpy, double, nb::shape<4, 4>>(double_array, 2, shape, deleter);
                   });
}

void define_inverse_kinematics(nb::module_ &robot_module, std::function<vector<Matrix1x6>(Matrix4x4)> ik_function) {
  robot_module.def("inverse_kinematics", [=](nb::ndarray<> tensor) {
    // Copy the received tensor to a row-major Eigen matrix
    Matrix4x4 rowMajorMatrix;
    memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

    // Call the IK function
    vector<Matrix1x6> solutions = ik_function(rowMajorMatrix);

    // Copy returned Matrices into tensors
    using np_array_1x6 = nb::ndarray<nb::numpy, double, nb::shape<1, 6>>;
    vector<np_array_1x6> vector_numpy;
    for (auto solution : solutions) {
      size_t shape[2] = {1, 6};
      double *double_array = new double[6];
      memcpy(double_array, solution.data(), 6 * sizeof(double));
      nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });
      auto tensor = np_array_1x6(double_array, 2, shape, deleter);
      vector_numpy.push_back(tensor);
    }

    return vector_numpy;
  });
}

void define_inverse_kinematics_closest(
    nb::module_ &robot_module,
    std::function<vector<Matrix1x6>(Matrix4x4, double, double, double, double, double, double)> ik_closest_function) {
  robot_module.def("inverse_kinematics_closest",
                   [=](nb::ndarray<> tensor,
                       double theta1,
                       double theta2,
                       double theta3,
                       double theta4,
                       double theta5,
                       double theta6) {
                     // Copy the received tensor to a row-major Eigen matrix
                     Matrix4x4 rowMajorMatrix;
                     memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

                     // Call the IK function
                     vector<Matrix1x6> solutions = ik_closest_function(
                         rowMajorMatrix, theta1, theta2, theta3, theta4, theta5, theta6);

                     // Copy returned Matrices into tensors
                     using np_array_1x6 = nb::ndarray<nb::numpy, double, nb::shape<1, 6>>;
                     vector<np_array_1x6> vector_numpy;
                     for (auto solution : solutions) {
                       size_t shape[2] = {1, 6};
                       double *double_array = new double[6];
                       memcpy(double_array, solution.data(), 6 * sizeof(double));
                       nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });
                       auto tensor = np_array_1x6(double_array, 2, shape, deleter);
                       vector_numpy.push_back(tensor);
                     }

                     return vector_numpy;
                   });
}

void define_inverse_kinematics_with_tcp(nb::module_ &robot_module,
                                        std::function<vector<Matrix1x6>(Matrix4x4, Matrix4x4)> ik_with_tcp_function) {
  robot_module.def("inverse_kinematics_with_tcp", [=](nb::ndarray<> tensor, nb::ndarray<> tcp_transform) {
    // Copy the received tensor to a row-major Eigen matrix
    Matrix4x4 rowMajorMatrix;
    memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

    // Copy the received tcp_transform to a row-major Eigen matrix
    Matrix4x4 tcp_transform_eigen;
    memcpy(tcp_transform_eigen.data(), tcp_transform.data(), 16 * sizeof(double));

    // Call the IK function
    vector<Matrix1x6> solutions = ik_with_tcp_function(rowMajorMatrix, tcp_transform_eigen);

    // Copy returned Matrices into tensors
    using np_array_1x6 = nb::ndarray<nb::numpy, double, nb::shape<1, 6>>;
    vector<np_array_1x6> vector_numpy;
    for (auto solution : solutions) {
      size_t shape[2] = {1, 6};
      double *double_array = new double[6];
      memcpy(double_array, solution.data(), 6 * sizeof(double));
      nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });
      auto tensor = np_array_1x6(double_array, 2, shape, deleter);
      vector_numpy.push_back(tensor);
    }

    return vector_numpy;
  });
}

void define_inverse_kinematics_closest_with_tcp(
    nb::module_ &robot_module,
    std::function<vector<Matrix1x6>(Matrix4x4, Matrix4x4, double, double, double, double, double, double)>
        ik_closest_with_tcp_function) {
  robot_module.def("inverse_kinematics_closest_with_tcp",
                   [=](nb::ndarray<> tensor,
                       nb::ndarray<> tcp_transform,
                       double theta1,
                       double theta2,
                       double theta3,
                       double theta4,
                       double theta5,
                       double theta6) {
                     // Copy the received tensor to a row-major Eigen matrix
                     Matrix4x4 rowMajorMatrix;
                     memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

                     // Copy the received tcp_transform to a row-major Eigen matrix
                     Matrix4x4 tcp_transform_eigen;
                     memcpy(tcp_transform_eigen.data(), tcp_transform.data(), 16 * sizeof(double));

                     // Call the IK function
                     vector<Matrix1x6> solutions = ik_closest_with_tcp_function(
                         rowMajorMatrix, tcp_transform_eigen, theta1, theta2, theta3, theta4, theta5, theta6);

                     // Copy returned Matrices into tensors
                     using np_array_1x6 = nb::ndarray<nb::numpy, double, nb::shape<1, 6>>;
                     vector<np_array_1x6> vector_numpy;
                     for (auto solution : solutions) {
                       size_t shape[2] = {1, 6};
                       double *double_array = new double[6];
                       memcpy(double_array, solution.data(), 6 * sizeof(double));
                       nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });
                       auto tensor = np_array_1x6(double_array, 2, shape, deleter);
                       vector_numpy.push_back(tensor);
                     }

                     return vector_numpy;
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
