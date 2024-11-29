from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the coffee order handler
    coffee_handler_node = Node(
        package='robot_trajectory_manipulation',
        executable='coffee_order_handler_real',
        name='coffee_order_handler_real',
        output='screen',
        parameters=[],  # Add any parameters if needed
        remappings=[]   # Add any topic remappings if needed
    )

    return LaunchDescription([
        coffee_handler_node
    ])