from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id_arg = DeclareLaunchArgument(
        'joy_device_id',
        default_value='0',
        description='Joystick device ID (run joy_enumerate_devices to list available devices)',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_id': LaunchConfiguration('joy_device_id')}],
    )

    teleop_node = Node(
        package='hexapod_gazebo',
        executable='joy_teleop',
        name='joy_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        device_id_arg,
        joy_node,
        teleop_node,
    ])