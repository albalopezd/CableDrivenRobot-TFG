#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cable_robot_kinematics/pcc_kinematics.hpp"


using namespace std::chrono_literals;

class IKBridge : public rclcpp::Node
{
    public:
        IKBridge() : Node("ik_bridge"), n_segment_(4), d_cable_(0.11),
        segment_spacing_(0.05675),
        phi_cable_1_(M_PI), phi_cable_2_(M_PI / 2.0), phi_cable_3_(-M_PI / 2.0)
        {
            cable_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("cable_joint_commands", 10);
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

            
            double L_total = n_segment_ * segment_spacing_;
            double z_arc = z + L_total;

            auto [bx_unused, by_unused, theta, phi] = cable_robot_kinematics::per_segment_bending(x, y, z_arc, n_segment_);
            (void)bx_unused; (void)by_unused;

            auto [c1, c2, c3] = cable_robot_kinematics::get_cable_pull(theta, phi, d_cable_, phi_cable_1_,
                phi_cable_2_, phi_cable_3_);

            auto stamp = this->now();

            sensor_msgs::msg::JointState cable_msg;
            cable_msg.header.stamp = stamp;
            cable_msg.name = {"cable_1_joint", "cable_2_joint", "cable_3_joint"};
            cable_msg.position = {c1, c2, c3};

            cable_publisher_->publish(cable_msg);
        }

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cable_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bend_sub_;

        int n_segment_;
        double d_cable_;
        double segment_spacing_;
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