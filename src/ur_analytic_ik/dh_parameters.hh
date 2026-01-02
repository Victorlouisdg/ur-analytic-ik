#pragma once
#include <math.h>

// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

namespace ur {
const double alpha1 = M_PI_2;
const double alpha4 = M_PI_2;
const double alpha5 = -M_PI_2;
}  // namespace ur
#pragma once

namespace ur3 {
const double d1 = 0.1519;
const double d4 = 0.11235;
const double d5 = 0.08535;
const double d6 = 0.0819;
const double a2 = -0.24365;
const double a3 = -0.21325;
}  // namespace ur3

namespace ur3e {
const double d1 = 0.15185;
const double d4 = 0.13105;
const double d5 = 0.08535;
const double d6 = 0.0921;
const double a2 = -0.24355;
const double a3 = -0.2132;
}  // namespace ur3e

namespace ur5 {
const double d1 = 0.089159;
const double d4 = 0.10915;
const double d5 = 0.09465;
const double d6 = 0.0823;
const double a2 = -0.425;
const double a3 = -0.39225;
}  // namespace ur5

namespace ur5e {
const double d1 = 0.1625;
const double d4 = 0.1333;
const double d5 = 0.0997;
const double d6 = 0.0996;
const double a2 = -0.425;
const double a3 = -0.3922;
}  // namespace ur5e

namespace ur7e {  // Note: UR7e is the same robot as the UR5e, UR renamed them in 2025
const double d1 = 0.1625;
const double d4 = 0.1333;
const double d5 = 0.0997;
const double d6 = 0.0996;
const double a2 = -0.425;
const double a3 = -0.3922;
}  // namespace ur7e

namespace ur8long {
const double d1 = 0.2186;
const double d4 = 0.1824;
const double d5 = 0.1361;
const double d6 = 0.1434;
const double a2 = -0.8989;
const double a3 = -0.7149;
}  // namespace ur8long

namespace ur10 {
const double d1 = 0.1273;
const double d4 = 0.163941;
const double d5 = 0.1157;
const double d6 = 0.0922;
const double a2 = -0.612;
const double a3 = -0.5723;
}  // namespace ur10

namespace ur10e {
const double d1 = 0.1807;
const double d4 = 0.17415;
const double d5 = 0.11985;
const double d6 = 0.11655;
const double a2 = -0.6127;
const double a3 = -0.57155;
}  // namespace ur10e

namespace ur12e {  // Note: UR12e is the same robot as the UR10e, UR renamed them in 2025
const double d1 = 0.1807;
const double d4 = 0.17415;
const double d5 = 0.11985;
const double d6 = 0.11655;
const double a2 = -0.6127;
const double a3 = -0.57155;
}  // namespace ur12e

namespace ur15 {
const double d1 = 0.2186;
const double d4 = 0.1824;
const double d5 = 0.1361;
const double d6 = 0.1434;
const double a2 = -0.6475;
const double a3 = -0.5164;
}  // namespace ur15

namespace ur18 {
const double d1 = 0.2186;
const double d4 = 0.1824;
const double d5 = 0.1361;
const double d6 = 0.1434;
const double a2 = -0.475;
const double a3 = -0.3389;
}  // namespace ur18

namespace ur20 {
const double d1 = 0.2363;
const double d4 = 0.2010;
const double d5 = 0.1593;
const double d6 = 0.1543;
const double a2 = -0.8620;
const double a3 = -0.7287;
}  // namespace ur20

namespace ur30 {
const double d1 = 0.2363;
const double d4 = 0.2010;
const double d5 = 0.1593;
const double d6 = 0.1543;
const double a2 = -0.6370;
const double a3 = -0.5037;
}  // namespace ur30
