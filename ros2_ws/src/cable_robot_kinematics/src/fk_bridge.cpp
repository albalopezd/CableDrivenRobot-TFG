#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class FKBridge : public rclcpp::Node
{
    public:
        FKBridge() : Node("fk_bridge"), x_scale_(1.0), y_scale_(1.0), z_scale_(1.0),
        z_offset_(0.1) 
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
            sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "cable_joint_states", 10, std::bind(&FKBridge::pose_callback,
                this, std::placeholders::_1));      
        }

    private:
        void pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "world";
            pose_msg.pose.position.x = msg->position[0] / x_scale_;
            pose_msg.pose.position.y = msg->position[1] / y_scale_;
            pose_msg.pose.position.z = msg->position[2] / z_scale_;
            pose_msg.pose.orientation.w = 1.0;

            publisher_->publish(pose_msg);
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

        double x_scale_;
        double y_scale_;
        double z_scale_;
        double z_offset_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<FKBridge>());
    rclcpp::shutdown();
    return 0;
}