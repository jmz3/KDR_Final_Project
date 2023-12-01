# Launch File for Simulating the UR5 for RDKDC

# Author: Jakub Piwowarczyk, 08/27/2023
# UPDATE: Jeremy Zhang 11/20/2023

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)  # , ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
)  # , FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# from launch.conditions import IfCondition


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5_kdr"), "rviz", "view_robot.rviz"]
    )

    # velocity_control = LaunchConfiguration('velocity_control')
    #
    # velocity_control_arg = DeclareLaunchArgument(
    #    'velocity_control',
    #    default_value="False"
    # )

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
                    [FindPackageShare("ur_bringup"), "launch", "ur_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5",
            "robot_ip": "yyy.yyy.yyy.yyy",
            "initial_joint_controller": "joint_trajectory_controller",
            "use_fake_hardware": "true",
            "launch_rviz": "False",
        }.items(),
    )

    kdr_frame_node = Node(
        package="ur5_kdr",
        namespace="",
        executable="frame_publisher",
        name="frame_pub_node",
    )

    # rdkdc_control_node = Node(
    #        package='rdkdc',
    #        namespace='',
    #        executable='joint_pos_ctrl',
    #        name='joint_pos_node'
    #    )

    # rdkdc_vel_node = Node(
    #        package='rdkdc',
    #        namespace='',
    #        executable='joint_vel_ctrl',
    #        name='joint_vel_node',
    #        condition=IfCondition(velocity_control)
    #    )
    #
    # load_vel_controller = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=[
    #        "forward_velocity_controller",
    #        "-c",
    #        "/controller_manager",
    #        "--controller-manager-timeout",
    #        "10",
    #        "--inactive",
    #    ],
    #    condition=IfCondition(velocity_control)
    # )

    return LaunchDescription(
        [
            rviz_node,
            ur_driver_launch,
            kdr_frame_node  # ,
            # rdkdc_vel_node,
            # load_vel_controller
            # rdkdc_control_node
        ]
    )
