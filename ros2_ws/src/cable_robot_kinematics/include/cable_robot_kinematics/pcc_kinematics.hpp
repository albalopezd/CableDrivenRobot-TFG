#pragma once

#include <cmath>
#include <tuple>

namespace cable_robot_kinematics
{
    struct IKResult
    {
        double bx_motor;
        double by_motor;
        double theta;
        double phi;
    };

    struct FKResult
    {
        double x;
        double y;
        double z;
        double theta;
        double phi;
    };

    IKResult inv_kinematics(double x, double y, double z);

    std::tuple<double, double, double> get_cable_pull(
        double theta_total, double phi, double d_cable,
        double phi_cable_1 = M_PI, double phi_cable_2 = M_PI / 2.0, double phi_cable_3 = -M_PI / 2.0
    );

    std::tuple<double, double, double, double> per_segment_bending(
        double x, double y, double z, int n_segment
    );

    FKResult fwd_kinematics(
        double c1, double c2, double c3,
        double L_total,
        double d_cable = 0.02
    );

}  // namespace cable_robot_kinematics