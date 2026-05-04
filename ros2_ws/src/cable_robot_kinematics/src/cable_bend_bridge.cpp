#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class CableBendBridge : public rclcpp::Node
{
    public:
        CableBendBridge() : Node("cable_bend_bridge"), n_segment_(4), d_cable_(0.02)
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("bend_joint_commands", 10);
            manual_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "manual_cable_joint_commands", 10, std::bind(&CableBendBridge::joint_state_callback,
                this, std::placeholders::_1));
        }

    private:
        
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            double c1 = 0.0;
            double c2 = 0.0;
            double c3 = 0.0;

            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                if (msg->name[i] == "cable_1_joint") {
                c1 = msg->position[i];
                } else if (msg->name[i] == "cable_2_joint") {
                c2 = msg->position[i];
                } else if (msg->name[i] == "cable_3_joint") {
                c3 = msg->position[i];
                }
            }

            if (n_segment_ <= 0.0 || d_cable_ == 0.0) return;

            double scale = d_cable_ * n_segment_ * n_segment_;
            double bx_segment = (c1 - c2) / (2.0 * scale);
            double by_segment = -c3 / scale;

            sensor_msgs::msg::JointState bend_msg;
            bend_msg.header.stamp = this->now();
            bend_msg.name = {
                "joint_bend_x", "joint_bend_y",
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
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manual_sub_;

        int n_segment_;
        double d_cable_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CableBendBridge>());
    rclcpp::shutdown();
    return 0;
}