from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for IMU puck'
        ),

        Node(
            package='imu_driver',
            executable='imu_driver',
            name='IMUDriver',
            output='screen',
            parameters=[{
                'port' : LaunchConfiguration('port')
                }]
        )
    ])