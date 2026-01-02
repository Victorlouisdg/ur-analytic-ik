#pragma once
#include <math.h>

// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

namespace ur {
const double alpha1 = M_PI_2;
const double alpha4 = M_PI_2;
const double alpha5 = -M_PI_2;
}  // namespace ur


namespace ur3 {
    struct Args {
        static constexpr double d1 = 0.1519;
        static constexpr double d4 = 0.11235;
        static constexpr double d5 = 0.08535;
        static constexpr double d6 = 0.0819;
        static constexpr double a2 = -0.24365;
        static constexpr double a3 = -0.21325;
    };
}  // namespace ur3

namespace ur3e {
    struct Args {
        static constexpr double d1 = 0.15185;
        static constexpr double d4 = 0.13105;
        static constexpr double d5 = 0.08535;
        static constexpr double d6 = 0.0921;
        static constexpr double a2 = -0.24355;
        static constexpr double a3 = -0.2132;
    };
}  // namespace ur3e


namespace ur5 {
    struct Args {
        static constexpr double d1 = 0.089159;
        static constexpr double d4 = 0.10915;
        static constexpr double d5 = 0.09465;
        static constexpr double d6 = 0.0823;
        static constexpr double a2 = -0.425;
        static constexpr double a3 = -0.39225;
    };
}  // namespace ur5

namespace ur5e {
    struct Args {
        static constexpr double d1 = 0.1625;
        static constexpr double d4 = 0.1333;
        static constexpr double d5 = 0.0997;
        static constexpr double d6 = 0.0996;
        static constexpr double a2 = -0.425;
        static constexpr double a3 = -0.3922;
    };
}  // namespace ur5e

namespace ur7e {  // Note: UR7e is the same robot as the UR5e, UR renamed them in 2025
    struct Args {
        static constexpr double d1 = 0.1625;
        static constexpr double d4 = 0.1333;
        static constexpr double d5 = 0.0997;
        static constexpr double d6 = 0.0996;
        static constexpr double a2 = -0.425;
        static constexpr double a3 = -0.3922;
    };
}  // namespace ur7e

namespace ur8long {
    struct Args {
        static constexpr double d1 = 0.2186;
        static constexpr double d4 = 0.1824;
        static constexpr double d5 = 0.1361;
        static constexpr double d6 = 0.1434;
        static constexpr double a2 = -0.8989;
        static constexpr double a3 = -0.7149;
    }; 
}  // namespace ur8long

namespace ur10 {
    struct Args {
        static constexpr double d1 = 0.1273;
        static constexpr double d4 = 0.163941;
        static constexpr double d5 = 0.1157;
        static constexpr double d6 = 0.0922;
        static constexpr double a2 = -0.612;
        static constexpr double a3 = -0.5723;
    };
}  // namespace ur10

namespace ur10e {
    struct Args {
        static constexpr double d1 = 0.1807;
        static constexpr double d4 = 0.17415;
        static constexpr double d5 = 0.11985;
        static constexpr double d6 = 0.11655;
        static constexpr double a2 = -0.6127;
        static constexpr double a3 = -0.57155;
    };
}  // namespace ur10e

namespace ur12e {  // Note: UR12e is the same robot as the UR10e, UR renamed them in 2025
    struct Args {
        static constexpr double d1 = 0.1807;
        static constexpr double d4 = 0.17415;
        static constexpr double d5 = 0.11985;
        static constexpr double d6 = 0.11655;
        static constexpr double a2 = -0.6127;
        static constexpr double a3 = -0.57155;
    };
}  // namespace ur12e

namespace ur15 {
    struct Args {
        static constexpr double d1 = 0.2186;
        static constexpr double d4 = 0.1824;
        static constexpr double d5 = 0.1361;
        static constexpr double d6 = 0.1434;
        static constexpr double a2 = -0.6475;
        static constexpr double a3 = -0.5164;
    };
}  // namespace ur15

namespace ur16e {
    struct Args {
        static constexpr double d1 = 0.1807;
        static constexpr double d4 = 0.17415;
        static constexpr double d5 = 0.11985;
        static constexpr double d6 = 0.11655;
        static constexpr double a2 = -0.4784;
        static constexpr double a3 = -0.36;
    };
}  // namespace ur16e

namespace ur18 {
    struct Args {
        static constexpr double d1 = 0.2186;
        static constexpr double d4 = 0.1824;
        static constexpr double d5 = 0.1361;
        static constexpr double d6 = 0.1434;
        static constexpr double a2 = -0.475;
        static constexpr double a3 = -0.3389;
    };
}  // namespace ur18

namespace ur20 {
    struct Args {
        static constexpr double d1 = 0.2363;
        static constexpr double d4 = 0.2010;
        static constexpr double d5 = 0.1593;
        static constexpr double d6 = 0.1543;
        static constexpr double a2 = -0.8620;
        static constexpr double a3 = -0.7287;
    };
}  // namespace ur20

namespace ur30 {
    struct Args {
        static constexpr double d1 = 0.2363;
        static constexpr double d4 = 0.2010;
        static constexpr double d5 = 0.1593;
        static constexpr double d6 = 0.1543;
        static constexpr double a2 = -0.6370;
        static constexpr double a3 = -0.5037;
    };
}  // namespace ur30
