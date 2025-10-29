from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    bringup_lidar_dir = get_package_share_directory('oradar_lidar')
    launch_lidar_dir = os.path.join(bringup_lidar_dir, 'launch')

    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_lidar_dir, 'ms200_scan.launch.py')),
    )

    bringup_scan_matcher_dir = get_package_share_directory('ros2_laser_scan_matcher')
    launch_scan_matcher_dir = os.path.join(bringup_scan_matcher_dir, 'launch')

    wheeltec_scan_matcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_scan_matcher_dir, 'laser_scan_matcher_launch.py')),
    )

    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar,
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
