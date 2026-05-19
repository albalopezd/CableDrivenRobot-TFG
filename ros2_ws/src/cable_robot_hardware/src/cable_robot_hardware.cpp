#include "cable_robot_hardware/cable_robot_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
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
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CableRobotHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    int spd0 = static_cast<int>(hw_velocities_[1] * 1000.0);
    int spd1 = static_cast<int>(hw_velocities_[0] * 1000.0);
    int spd2 = static_cast<int>(hw_velocities_[2] * 1000.0);

    // Communication protocol with Arduino
    std::string msg = "v:" + std::to_string(spd0) + "," +
                                std::to_string(spd1) + "," +
                                std::to_string(spd2) + "\n";
    bool ok = serial_write(msg);
    if (++log_count_ % 10 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("CableRobotHardware"),
            "write -> %s (ok=%d)", msg.c_str(), ok);
    }

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