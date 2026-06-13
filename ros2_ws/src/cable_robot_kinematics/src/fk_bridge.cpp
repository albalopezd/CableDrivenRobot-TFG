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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cable_robot_kinematics/pcc_kinematics.hpp"

class FKBridge : public rclcpp::Node
{
public:
    FKBridge() : Node("fk_bridge"),
        n_segment_(4), segment_spacing_(0.05675), d_cable_(0.11)
    {
        L_total_ = n_segment_ * segment_spacing_;

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FKBridge::joint_state_cb, this, std::placeholders::_1));
    }

private:
    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const std::array<std::string, 3> names = {
            "cable_1_joint", "cable_2_joint", "cable_3_joint"};

        double c[3];
        for (int i = 0; i < 3; ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), names[i]);
            if (it == msg->name.end()) return;
            c[i] = msg->position[std::distance(msg->name.begin(), it)];
        }

        auto fk = cable_robot_kinematics::fwd_kinematics(c[0], c[1], c[2], L_total_, d_cable_);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "world";
        pose.pose.position.x = fk.x;
        pose.pose.position.y = -fk.y;
        pose.pose.position.z = fk.z;
        pose.pose.orientation.w = 1.0;

        publisher_->publish(pose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

    int n_segment_;
    double segment_spacing_;
    double d_cable_;
    double L_total_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FKBridge>());
    rclcpp::shutdown();
    return 0;
}
