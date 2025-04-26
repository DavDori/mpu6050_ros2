from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('mpu6050driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
        default_value=os.path.join(
            share_dir, 'config', 'mpu6050.yaml'),
        description='Path to the ROS2 parameters file to use.')

    mpu6050driver_node = Node(
        package='mpu6050',
        executable='mpu6050node',
        name='mpu6050node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    ld.add_action(params_declare)
    ld.add_action(mpu6050driver_node)
    return ld
