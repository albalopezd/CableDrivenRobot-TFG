#include "cable_robot_kinematics/pcc_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace cable_robot_kinematics
{
    IKResult inv_kinematics(double x, double y, double z)
    {
        double phi = 0.0;
        double theta = 0.0;

        if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) {
            phi = 0.0;
            theta = 0.0;
        } else {
            phi = std::atan2(y, x);
            const double xy_norm = std::sqrt(x * x + y * y);
            const double k = (2.0 * xy_norm) / (x * x + y * y + z * z);
            double acos_arg = 1.0 - (k * xy_norm);
            acos_arg = std::clamp(acos_arg, -1.0, 1.0);

            if (z > 0.0) {
                theta = std::acos(acos_arg);
            } else {
                theta = (2.0 * 3.1415) - std::acos(acos_arg);
            } 
        }

        return {-theta * std::sin(phi), theta * std::cos(phi), theta, phi};
    }

    std::tuple<double, double, double> get_cable_pull(
        double theta_total, double phi, double d_cable,
        double phi_cable_1, double phi_cable_2, double phi_cable_3) 
    {
        return {d_cable * theta_total * std::sin(phi + phi_cable_1),
                d_cable * theta_total * std::sin(phi + phi_cable_2),
                d_cable * theta_total * std::sin(phi + phi_cable_3)};
    }

    std::tuple<double, double, double, double> per_segment_bending(
        double x, double y, double z, int n_segment)
    {
        const auto ik = inv_kinematics(x, y, z);
        return {ik.bx_motor / n_segment, ik.by_motor / n_segment, ik.theta, ik.phi};
    }

    FKResult fwd_kinematics(
        double c1, double c2, double c3,
        double L_total, double d_cable)
    {
        const double sin_comp = -c1;
        const double cos_comp = (std::abs(c2) >= std::abs(c3)) ? c2 : -c3;

        const double theta = std::sqrt(sin_comp*sin_comp + cos_comp*cos_comp) / d_cable;
        const double phi = std::atan2(sin_comp, cos_comp);

        if (theta < 1e-6) {
            return {0.0, 0.0, 0.0, theta, phi};
        }

        const double R = L_total / theta;
        return {
            R * (1.0 - std::cos(theta)) * std::cos(phi),
            R * (1.0 - std::cos(theta)) * std::sin(phi),
            R * std::sin(theta) - L_total,
            theta,
            phi
        };
    }

}