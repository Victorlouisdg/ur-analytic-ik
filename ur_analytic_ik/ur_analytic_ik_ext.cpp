#include "forward_kinematics.hh"
#include "inverse_kinematics.hh"
#include <nanobind/nanobind.h>
#include <nanobind/tensor.h>

namespace nb = nanobind;

using namespace nb::literals;

NB_MODULE(ur_analytic_ik_ext, m) {
  m.def(
      "ur5e_forward_kinematics",
      [](double theta1, double theta2, double theta3, double theta4,
         double theta5, double theta6) {
        size_t shape[2] = {4, 4};

        Matrix4x4 rowMajorMatrix = ur5e_forward_kinematics(
            theta1, theta2, theta3, theta4, theta5, theta6);

        double *double_array = new double[16];
        memcpy(double_array, rowMajorMatrix.data(), 16 * sizeof(double));
        nb::capsule deleter(
            double_array, [](void *data) noexcept { delete[](double *) data; });

        return nb::tensor<nb::numpy, double, nb::shape<4, 4>>(double_array, 2,
                                                              shape, deleter);
      },
      "theta1"_a, "theta2"_a, "theta3"_a, "theta4"_a, "theta5"_a, "theta6"_a);

  m.def("ur5e_inverse_kinematics", [](nb::tensor<> tensor) {
    // Copy the received tensor to a row-major Eigen matrix
    Matrix4x4 rowMajorMatrix;
    memcpy(rowMajorMatrix.data(), tensor.data(), 16 * sizeof(double));

    // Call the IK function
    Matrix8x6 solutions = ur5e_inverse_kinematics(rowMajorMatrix);

    // Copy returned Matrix to array of doubles
    size_t shape[2] = {8, 6};
    double *double_array = new double[48];
    memcpy(double_array, solutions.data(), 48 * sizeof(double));
    nb::capsule deleter(
        double_array, [](void *data) noexcept { delete[](double *) data; });

    return nb::tensor<nb::numpy, double, nb::shape<8, 6>>(double_array, 2,
                                                          shape, deleter);
  });

    m.def("inspect", [](nb::tensor<> tensor) {
    printf("Tensor data pointer : %p\n", tensor.data());
    printf("Tensor dimension : %zu\n", tensor.ndim());
    for (size_t i = 0; i < tensor.ndim(); ++i) {
      printf("Tensor dimension [%zu] : %zu\n", i, tensor.shape(i));
      printf("Tensor stride    [%zu] : %zd\n", i, tensor.stride(i));
    }
    printf("Device ID = %u (cpu=%i, cuda=%i)\n", tensor.device_id(),
           int(tensor.device_type() == nb::device::cpu::value),
           int(tensor.device_type() == nb::device::cuda::value));
    printf("Tensor dtype check: int16=%i, uint32=%i, float32=%i, double=%i\n",
           tensor.dtype() == nb::dtype<int16_t>(),
           tensor.dtype() == nb::dtype<uint32_t>(),
           tensor.dtype() == nb::dtype<float>(),
           tensor.dtype() == nb::dtype<double>());
  });
}
