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

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class HardwareController : public rclcpp::Node
{
public:
    HardwareController() : Node("hardware_controller")
    {
        kp_              = this->declare_parameter<double>("kp", 5.0);
        max_vel_         = this->declare_parameter<double>("max_vel", 1.0);
        min_vel_         = this->declare_parameter<double>("min_vel", 0.15);
        min_vel_release_ = this->declare_parameter<double>("min_vel_release", 0.185);
        deadband_        = this->declare_parameter<double>("deadband", 0.001);
        desired_min_     = this->declare_parameter<double>("desired_min", -0.015);

        desired_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "manual_cable_joint_commands", 10,
            std::bind(&HardwareController::desired_cb, this, std::placeholders::_1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&HardwareController::current_cb, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/cable_velocity_controller/commands", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&HardwareController::timer_loop, this));
    }

private:
    void desired_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < JOINTS.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), JOINTS[i]);
            if (it != msg->name.end()) {
                size_t idx = std::distance(msg->name.begin(), it);
                desired_[i] = std::max(desired_min_, msg->position[idx]);
            }
        }
    }

    void current_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < JOINTS.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), JOINTS[i]);
            if (it != msg->name.end()) {
                size_t idx = std::distance(msg->name.begin(), it);
                current_[i] = msg->position[idx];
            }
        }
    }

    void timer_loop()
    {
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.resize(JOINTS.size());

        for (size_t i = 0; i < JOINTS.size(); ++i) {
            double error = desired_[i] - current_[i];
            double vel = 0.0;
            if (std::abs(error) >= deadband_) {
                vel = kp_ * error;
                vel = std::max(-max_vel_, std::min(max_vel_, vel));
                double min_v = (vel < 0.0) ? min_vel_release_ : min_vel_;
                if (std::abs(vel) < min_v) {
                    vel = min_v * (vel > 0.0 ? 1.0 : -1.0);
                }
            }
            cmd.data[i] = vel;
        }
        cmd_pub_->publish(cmd);
    }

    static const std::array<std::string, 3> JOINTS;

    double kp_;
    double max_vel_;
    double min_vel_;
    double min_vel_release_;
    double deadband_;
    double desired_min_;

    std::array<double, 3> desired_ {0.0, 0.0, 0.0};
    std::array<double, 3> current_ {0.0, 0.0, 0.0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

const std::array<std::string, 3> HardwareController::JOINTS = {
    "cable_1_joint", "cable_2_joint", "cable_3_joint"};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareController>());
    rclcpp::shutdown();
    return 0;
}
