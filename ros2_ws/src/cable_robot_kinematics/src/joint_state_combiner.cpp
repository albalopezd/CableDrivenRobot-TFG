#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateCombiner : public rclcpp::Node
{
    public:
        JointStateCombiner() : Node("joint_state_combiner")
        {   
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            bend_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "bend_joint_commands", 10, std::bind(&JointStateCombiner::joint_state_callback,
                this, std::placeholders::_1));
            cable_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "cable_joint_commands", 10, std::bind(&JointStateCombiner::joint_state_callback,
                this, std::placeholders::_1));
            manual_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "manual_cable_joint_commands", 10, std::bind(&JointStateCombiner::joint_state_callback,
                this, std::placeholders::_1));
            
            timer_ = this->create_wall_timer(100ms, std::bind(&JointStateCombiner::publish_state, this));
        }
    
    private:
        
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                std::string & name = msg->name[i];

                if (name == "joint_bend_x") joint_bend_x_ = msg->position[i];
                else if (name == "joint_bend_y") joint_bend_y_ = msg->position[i];
                else if (name == "joint_bend_x_1") joint_bend_x_1_ = msg->position[i];
                else if (name == "joint_bend_y_1") joint_bend_y_1_ = msg->position[i];
                else if (name == "joint_bend_x_2") joint_bend_x_2_ = msg->position[i];
                else if (name == "joint_bend_y_2") joint_bend_y_2_ = msg->position[i];
                else if (name == "joint_bend_x_3") joint_bend_x_3_ = msg->position[i];
                else if (name == "joint_bend_y_3") joint_bend_y_3_ = msg->position[i];
                else if (name == "cable_1_joint") cable_1_joint_ = msg->position[i];
                else if (name == "cable_2_joint") cable_2_joint_ = msg->position[i];
                else if (name == "cable_3_joint") cable_3_joint_ = msg->position[i];
            }
            publish_state();
        }

        void publish_state()
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->now();
            msg.name = {
                "joint_bend_x", "joint_bend_y",
                "joint_bend_x_1", "joint_bend_y_1",
                "joint_bend_x_2", "joint_bend_y_2",
                "joint_bend_x_3", "joint_bend_y_3",
                "cable_1_joint", "cable_2_joint", "cable_3_joint"
            };
            msg.position = {
                joint_bend_x_, joint_bend_y_,
                joint_bend_x_1_, joint_bend_y_1_,
                joint_bend_x_2_, joint_bend_y_2_,
                joint_bend_x_3_, joint_bend_y_3_,
                cable_1_joint_, cable_2_joint_, cable_3_joint_
            };
            publisher_->publish(msg);
        }

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr bend_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cable_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manual_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        double joint_bend_x_ = 0.0;
        double joint_bend_y_ = 0.0;
        double joint_bend_x_1_ = 0.0;
        double joint_bend_y_1_ = 0.0;
        double joint_bend_x_2_ = 0.0;
        double joint_bend_y_2_ = 0.0;
        double joint_bend_x_3_ = 0.0;
        double joint_bend_y_3_ = 0.0;
        double cable_1_joint_ = 0.0;
        double cable_2_joint_ = 0.0;
        double cable_3_joint_ = 0.0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<JointStateCombiner>());
    rclcpp::shutdown();
    return 0;
}