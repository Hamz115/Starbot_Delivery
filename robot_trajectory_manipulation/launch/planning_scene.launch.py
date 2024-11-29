import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="sim_moveit_config").to_moveit_configs()

    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        package='robot_trajectory_manipulation',
        executable='planning_scene_node',
        name='planning_scene',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        moveit_cpp_node
    ])