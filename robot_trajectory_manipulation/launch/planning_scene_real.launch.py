import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="sim_moveit_config").to_moveit_configs()

    # Launch the pick and place node
    pick_place_node = Node(
        package='robot_trajectory_manipulation',
        executable='planning_scene_real',
        name='planning_scene',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ]
    )

    return LaunchDescription([
        pick_place_node
    ])