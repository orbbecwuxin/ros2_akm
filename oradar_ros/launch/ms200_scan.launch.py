#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/wheeltec_controller'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 15}
      ]
'''

def generate_launch_description():
    # LiDAR publisher node
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',  # 修改这里
        name='MS200',              # 修改这里
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'laser_frame'},
            {'scan_topic': '/MS200/scan'},
            {'port_name': '/dev/wheeltec_lidar'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 15}
        ]
    )

    # base_link to laser_frame tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',  # 修改这里
        name='base_link_to_base_laser',           # 修改这里
        arguments=['0.11', '0', '0.08', '3.14', '0', '0', 'base_link', 'laser_frame']
    )


    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('oradar_lidar'),
                'config',
                'laser_filter_config.yaml'
            )
        ],
        remappings=[
            ('scan', 'MS200/scan'),
            ('scan_filtered', 'scan')
        ]
    )

    # Define LaunchDescription variable
    ord = LaunchDescription()
    ord.add_action(laser_filter_node)  # 添加滤波器节点

    ord.add_action(ordlidar_node)
    ord.add_action(base_link_to_laser_tf_node)

    return ord