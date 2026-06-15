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

#include "cable_robot_hardware/cable_robot_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace cable_robot_hardware
{

hardware_interface::CallbackReturn CableRobotHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
    { 
        return hardware_interface::CallbackReturn::ERROR;
    } 
  
    port_ = info_.hardware_parameters.at("port");
    baud_ = std::stoi(info_.hardware_parameters.at("baud"));
    cable_speed_ = std::stod(info_.hardware_parameters.at("cable_speed"));

    hw_positions_.assign(info_.joints.size(), 0.0);
    hw_velocities_.assign(info_.joints.size(), 0.0);

    node_ = rclcpp::Node::make_shared("cable_robot_hardware_node");
    voltage_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/servo_voltage", 10);

    return hardware_interface::CallbackReturn::SUCCESS;
} 

hardware_interface::CallbackReturn CableRobotHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CableRobotHardware"),
        "No se pudo abrir %s", port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tty;
    tcgetattr(serial_fd_, &tty);
    speed_t speed = (baud_ == 115200) ? B115200 : B9600;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tcsetattr(serial_fd_, TCSANOW, &tty);

    RCLCPP_INFO(rclcpp::get_logger("CableRobotHardware"),
        "Puerto %s abierto a %d baud", port_.c_str(), baud_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CableRobotHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    serial_write("v:0,0,0\n");
    RCLCPP_INFO(rclcpp::get_logger("CableRobotHardware"), "Hardware activado");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CableRobotHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    serial_write("v:0,0,0\n");
    close(serial_fd_);
    RCLCPP_INFO(rclcpp::get_logger("CableRobotHardware"), "Hardware desactivado");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
    CableRobotHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(
        info_.joints[i].name, "position", &hw_positions_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
    CableRobotHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_velocities_[i]);
    }
    return interfaces;
}

hardware_interface::return_type CableRobotHardware::read(
    const rclcpp::Time &, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
        hw_positions_[i] += hw_velocities_[i] * cable_speed_ * period.seconds();
        if (hw_positions_[i] < 0.0) hw_positions_[i] = 0.0;
    }
    serial_read_voltages();
    return hardware_interface::return_type::OK;
}

void CableRobotHardware::serial_read_voltages()
{
    char buf[64];
    ssize_t n = ::read(serial_fd_, buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';
    serial_read_buf_ += buf;

    size_t pos;
    while ((pos = serial_read_buf_.find('\n')) != std::string::npos) {
        std::string line = serial_read_buf_.substr(0, pos);
        serial_read_buf_ = serial_read_buf_.substr(pos + 1);

        if (line.rfind("w:", 0) != 0) continue;

        std::istringstream ss(line.substr(2));
        std::string tok;
        std_msgs::msg::Float32MultiArray msg;
        while (std::getline(ss, tok, ',')) {
            int v = std::stoi(tok);
            msg.data.push_back(v > 0 ? static_cast<float>(v) / 1000.0f : 0.0f);
        }
        if (msg.data.size() == 3) {
            voltage_pub_->publish(msg);
        }
    }
}

hardware_interface::return_type CableRobotHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    int spd0 = static_cast<int>(hw_velocities_[1] * 1000.0);
    int spd1 = static_cast<int>(hw_velocities_[2] * 1000.0);
    int spd2 = static_cast<int>(-hw_velocities_[0] * 1000.0);

    std::string msg = "v:" + std::to_string(spd0) + "," +
                                std::to_string(spd1) + "," +
                                std::to_string(spd2) + "\n";
    serial_write(msg);

    return hardware_interface::return_type::OK;
}

bool CableRobotHardware::serial_write(const std::string & msg)
{
    ssize_t written = ::write(serial_fd_, msg.c_str(), msg.size());
    return written == static_cast<ssize_t>(msg.size());
}

}  // namespace cable_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
cable_robot_hardware::CableRobotHardware, hardware_interface::SystemInterface)
