# Launch File for Controlling the UR5 in Headless Mode for Kinematics & Dynamics of Robots

# Author: Jakub Piwowarczyk, 08/27/2023
# Author: Jeremy Zhang, 11/23/2023

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kdr"), "rviz", "view_robot.rviz"]
    )

    robot_ip = LaunchConfiguration("robot_ip")
    velocity_control = LaunchConfiguration("velocity_control")

    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="172.22.22.2")

    velocity_control_arg = DeclareLaunchArgument(
        "velocity_control", default_value="False"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_bringup"),
                        "launch",
                        "ur_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5",
            "robot_ip": robot_ip,
            "use_fake_hardware": "False",
            "launch_rviz": "False",
            "robot_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "True",
        }.items(),
    )

    # set_speed_cmd = ExecuteProcess(
    #    cmd=[[
    #        FindExecutable(name='ros2'),
    #        ' service call ',
    #        '/io_and_status_controller/set_speed_slider ',
    #        'ur_msgs/srv/SetSpeedSliderFraction ',
    #        'speed_slider_fraction:\\ 1.0'
    #    ]],
    #    shell=True
    # )

    kdr_frame_node = Node(
        package="ur5_kdr_custom",
        namespace="",
        executable="frame_publisher",
        name="frame_pub_node",
    )

    kdr_control_node = Node(
        package="ur5_kdr_custom",
        namespace="",
        executable="joint_pos_ctrl",
        name="joint_pos_node",
    )

    kdr_vel_node = Node(
        package="ur5_kdr_custom",
        namespace="",
        executable="joint_vel_ctrl",
        name="joint_vel_node",
        condition=IfCondition(velocity_control),
    )

    load_vel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_velocity_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
            "--inactive",
        ],
        condition=IfCondition(velocity_control),
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            velocity_control_arg,
            rviz_node,
            ur_driver_launch,
            # set_speed_cmd,
            kdr_frame_node,
            kdr_vel_node,
            load_vel_controller
            # kdr_control_node
        ]
    )
