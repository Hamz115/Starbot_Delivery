#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import OpaqueFunction


def gen_robot_info():

    pose_1 = [0.63, -0.76, -0.051]

    robot_name = "barista_1"
    x_pos = pose_1[0]
    y_pos = pose_1[1]
    z_pos = pose_1[2]
    robot = {'name': robot_name, 'x_pose': x_pos,
             'y_pose': y_pos, 'z_pose': z_pos, 'yaw_spawn': 0.0}

    print("############### ROBOTS MULTI ARRAY="+str(robot))

    return robot


def launch_setup(context, *args, **kwargs):

    launch_file_dir = os.path.join(get_package_share_directory(
        'barista_description'), 'launch')

    # Names and poses of the robots
    robot = gen_robot_info()

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_file_dir, 'spawn.launch.py')),
        launch_arguments={
            'x_spawn': TextSubstitution(text=str(robot['x_pose'])),
            'y_spawn': TextSubstitution(text=str(robot['y_pose'])),
            'yaw_spawn': TextSubstitution(text=str(robot['yaw_spawn'])),
            'entity_name': robot['name']
        }.items()))

    return [ld]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
