import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Launch the pick and place node
    moveit_cpp_node = Node(
        package='hole_detection_real_yolo',
        executable='hole_detector_real',
        name='hole_detector_real',
        output='screen',
        parameters=[
        ]
    )
    

    return LaunchDescription([
        moveit_cpp_node
    ])