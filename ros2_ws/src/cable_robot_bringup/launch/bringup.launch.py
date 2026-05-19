from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_share = get_package_share_directory("cable_robot_description")
    bringup_share = get_package_share_directory("cable_robot_bringup")

    xacro_file = os.path.join(description_share, "urdf", "cable_robot.urdf.xacro")
    rviz_config = os.path.join(description_share, "rviz", "view_robot.rviz")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Puerto USB del Arduino'
        ),
        DeclareLaunchArgument(
            'serial_baud',
            default_value='9600',
            description='Baud rate del Arduino'
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]),
        TimerAction(period=3.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["cable_velocity_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]),

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
        Node(
            package="cable_robot_kinematics",
            executable="hardware_controller.py",
            name="hardware_controller",
            output="screen",
        ),
        Node(
            package="cable_robot_kinematics",
            executable="cable_slider_gui.py",
            name="cable_slider_gui",
            output="screen",
        ),
    ])