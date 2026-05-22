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
