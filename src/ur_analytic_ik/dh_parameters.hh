#pragma once
#include <math.h>

// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

namespace ur {
const double alpha1 = M_PI_2;
const double alpha4 = M_PI_2;
const double alpha5 = -M_PI_2;
}  // namespace ur

namespace ur3e
{
    struct Args
    {
        static constexpr double d1 = 0.15185;
        static constexpr double d4 = 0.13105;
        static constexpr double d5 = 0.08535;
        static constexpr double d6 = 0.0921;
        static constexpr double a2 = -0.24355;
        static constexpr double a3 = -0.2132;
    };
}

namespace ur5e
{
    struct Args
    {
        static constexpr double d1 = 0.1625;
        static constexpr double d4 = 0.1333;
        static constexpr double d5 = 0.0997;
        static constexpr double d6 = 0.0996;
        static constexpr double a2 = -0.425;
        static constexpr double a3 = -0.3922;
    };
}

namespace ur10e
{
    struct Args
    {
        static constexpr double d1 = 0.1807;
        static constexpr double d4 = 0.17415;
        static constexpr double d5 = 0.11985;
        static constexpr double d6 = 0.11655;
        static constexpr double a2 = -0.6127;
        static constexpr double a3 = -0.57155;
    };
}
