import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()


    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        package='robot_trajectory_manipulation',
        executable='hole_placement__real_4',
        name='hole_placement_real_4',
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )