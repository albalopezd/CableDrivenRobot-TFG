// Copyright 2026 Alba López del Águila
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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