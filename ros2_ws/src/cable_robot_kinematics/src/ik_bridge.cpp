#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cable_robot_kinematics/continuum_kinematics.hpp"


using namespace std::chrono_literals;

class IKBridge : public rclcpp::Node
{
    public:
        IKBridge() : Node("ik_bridge"), n_segment_(4), d_cable_(0.02), phi_cable_1_(0.0),
        phi_cable_2_(2.09439), phi_cable_3_(4.18879) 
        {
            cable_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("cable_joint_commands", 10);
            bend_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("bend_joint_commands", 10);
            bend_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "desired_pose", 10, std::bind(&IKBridge::pose_callback,
                this, std::placeholders::_1));      
        }

    private:
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            double x = msg->pose.position.x;
            double y = msg->pose.position.y;
            double z = msg->pose.position.z;

            
            auto [bx_segment, by_segment, theta, phi] = cable_robot_kinematics::per_segment_bending(x, y, z, n_segment_);

            double theta_total = theta * n_segment_;

            auto [c1, c2, c3] = cable_robot_kinematics::get_cable_pull(theta_total, phi, d_cable_, phi_cable_1_, 
                phi_cable_2_, phi_cable_3_);

            auto stamp = this->now();

            sensor_msgs::msg::JointState cable_msg;
            cable_msg.header.stamp = stamp;
            cable_msg.name = {"cable_1_joint", "cable_2_joint", "cable_3_joint"};
            cable_msg.position = {c1, c2, c3};

            sensor_msgs::msg::JointState bend_msg;
            bend_msg.header.stamp = stamp;
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

            cable_publisher_->publish(cable_msg);
            bend_publisher_->publish(bend_msg);
        }

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cable_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr bend_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bend_sub_;

        int n_segment_;
        double d_cable_;
        double phi_cable_1_;
        double phi_cable_2_;
        double phi_cable_3_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<IKBridge>());
    rclcpp::shutdown();
    return 0;
}