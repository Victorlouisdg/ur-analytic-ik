#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include <nanobind/nanobind.h>
#include <nanobind/tensor.h>
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

                     // Copy returned Matrix to array of doubles
                     size_t shape[2] = {4, 4};
                     double *double_array = new double[16];
                     memcpy(double_array, rowMajorMatrix.data(), 16 * sizeof(double));
                     nb::capsule deleter(double_array, [](void *data) noexcept { delete[](double *) data; });

                     return nb::tensor<nb::numpy, double, nb::shape<4, 4>>(double_array, 2, shape, deleter);
                   });
}

void define_inverse_kinematics(nb::module_ &robot_module, std::function<vector<Matrix1x6>(Matrix4x4)> ik_function) {
  robot_module.def("inverse_kinematics", [=](nb::tensor<> tensor) {
    // Copy the received tensor to a row-major Eigen matrix
    Matrix4x4 rowMajorMatrix;
    memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

    // Call the IK function
    vector<Matrix1x6> solutions = ik_function(rowMajorMatrix);

    // Copy returned Matrices into tensors
    using np_array_1x6 = nb::tensor<nb::numpy, double, nb::shape<1, 6>>;
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
  define_inverse_kinematics(m_ur3e, ur3e::inverse_kinematics);
  define_forward_kinematics(m_ur3e, ur3e::forward_kinematics);

  nb::module_ m_ur5e = m.def_submodule("ur5e", "UR5e module");
  define_inverse_kinematics(m_ur5e, ur5e::inverse_kinematics);
  define_forward_kinematics(m_ur5e, ur5e::forward_kinematics);

  // This function is mostly still here to understand nanobind.
  // Once we properly handle tensors without copying etc, we can remove this.
  m.def("inspect", [](nb::tensor<> tensor) {
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
