#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mag',
            default_value='true',
            description='Use magnetometer data for orientation estimation'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for the IMU sensor'
        ),
        
        # MPU6500 Publisher Node (your existing publisher)
        Node(
            package='mpu6500_ros',
            executable='mpu6500_publisher',
            name='mpu6500_publisher',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': 50.0,
            }]
        ),
        
        # IMU Filter Madgwick Node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': LaunchConfiguration('use_mag'),
                'publish_tf': True,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
                'gain': 0.1,
                'zeta': 0.001,
                'mag_bias_x': 0.0,
                'mag_bias_y': 0.0,
                'mag_bias_z': 0.0,
                'orientation_stddev': 0.0,
            }]
        ),
        
        # Static transform publisher for IMU frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
    ])