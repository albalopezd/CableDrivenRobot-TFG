#pragma once

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

    IKResult inv_kinematics(double x, double y, double z);

    std::tuple<double, double, double> get_cable_pull(
        double theta_total, double phi, double d_cable,
        double phi_cable_1 = 0.0, double phi_cable_2 = 2.0 * 3.14 / 3.0, double phi_cable_3 = 4.0 * 3.14 / 3.0        
    );

    std::tuple<double, double, double, double> per_segment_bending(
        double x, double y, double z, int n_segment
    );

}  // namespace cable_robot_kinematics