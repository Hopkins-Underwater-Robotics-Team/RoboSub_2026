from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu6500_ros',
            executable='mpu6500_publisher',
            name='mpu6500_publisher',
            parameters=[{
                'frame_id': 'imu_link',
                'topic_imu': '/imu/data_raw',
                'topic_mag': '/imu/mag',
                'publish_rate': 100.0,
                'i2c_address': 0x68
            }],
            output='screen'
        )
    ])