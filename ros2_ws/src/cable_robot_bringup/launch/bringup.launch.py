from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_share = get_package_share_directory("cable_robot_description")
    xacro_file = os.path.join(description_share, "urdf", "cable_robot.urdf.xacro")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
        Node(
            package="cable_robot_kinematics",
            executable="ik_bridge_cpp",
            name="ik_bridge",
            output="screen",
        ),
        Node(
            package="cable_robot_kinematics",
            executable="fk_bridge_cpp",
            name="fk_bridge",
            output="screen",
        ),
        Node(
            package="cable_robot_kinematics",
            executable="joint_state_combiner_cpp",
            name="joint_state_combiner",
            output="screen",
        ),
        Node(
            package="cable_robot_kinematics",
            executable="cable_bend_bridge_cpp",
            name="cable_bend_bridge",
            output="screen",
        ),
    ])
