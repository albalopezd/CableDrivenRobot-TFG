#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class CableBendBridge : public rclcpp::Node
{
    public:
        CableBendBridge() : Node("cable_bend_bridge"), n_segment_(4), d_cable_(0.02)
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("bend_joint_commands", 10);
            joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&CableBendBridge::joint_state_callback,
                this, std::placeholders::_1));
        }

    private:

        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            double c1 = 0.0, c2 = 0.0, c3 = 0.0;
            bool found_c1 = false, found_c2 = false, found_c3 = false;

            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                if (msg->name[i] == "cable_1_joint") {
                    c1 = msg->position[i]; found_c1 = true;
                } else if (msg->name[i] == "cable_2_joint") {
                    c2 = msg->position[i]; found_c2 = true;
                } else if (msg->name[i] == "cable_3_joint") {
                    c3 = msg->position[i]; found_c3 = true;
                }
            }

            if (!found_c1 || !found_c2 || !found_c3) return;

            // FK model: phi_cable = {0, 2pi/3, 4pi/3}
            // sin_comp = c1, cos_comp = (c2-c3)/sqrt(3)
            const double sin_comp = c1;
            const double cos_comp = (c2 - c3) / std::sqrt(3.0);
            const double theta = std::sqrt(sin_comp * sin_comp + cos_comp * cos_comp) / d_cable_;
            const double phi   = std::atan2(sin_comp, cos_comp);

            // joint_bend_x+ → punta hacia -Y; joint_bend_y+ → punta hacia +X
            const double bx_segment = -theta * std::sin(phi) / n_segment_;
            const double by_segment =  theta * std::cos(phi) / n_segment_;

            sensor_msgs::msg::JointState bend_msg;
            bend_msg.header.stamp = this->now();
            bend_msg.name = {
                "joint_bend_x",   "joint_bend_y",
                "joint_bend_x_1", "joint_bend_y_1",
                "joint_bend_x_2", "joint_bend_y_2",
                "joint_bend_x_3", "joint_bend_y_3"
            };
            bend_msg.position = {
                bx_segment, by_segment,
                bx_segment, by_segment,
                bx_segment, by_segment,
                bx_segment, by_segment
            };
            publisher_->publish(bend_msg);
        }

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

        int n_segment_;
        double d_cable_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CableBendBridge>());
    rclcpp::shutdown();
    return 0;
}
