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

#ifndef CABLE_ROBOT_HARDWARE__CABLE_ROBOT_HARDWARE_HPP_
#define CABLE_ROBOT_HARDWARE__CABLE_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace cable_robot_hardware
{

class CableRobotHardware : public hardware_interface::SystemInterface
{
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams & params) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::string port_;
        int baud_;
        double cable_speed_;

        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        int serial_fd_;
        int log_count_ = 0;
        std::string serial_read_buf_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr voltage_pub_;

        bool serial_write(const std::string & msg);
        void serial_read_voltages();
};

}  // namespace cable_robot_hardware

#endif  // CABLE_ROBOT_HARDWARE__CABLE_ROBOT_HARDWARE_HPP_
