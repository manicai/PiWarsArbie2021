from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE = 'arbie'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=PACKAGE,
            executable='gamepad',
            name='gamepad',
        ),
        Node(
            package=PACKAGE,
            executable='control',
            name='control',
        ),
        Node(
            package=PACKAGE,
            executable='motors',
            name='motors',
        ),
        Node(
            package=PACKAGE,
            executable='launcher',
            name='launcher',
        ),
        Node(
            package=PACKAGE,
            executable='line_control',
            name='line_control',
        ),
        Node(
            package=PACKAGE,
            executable='line_sensor',
            name='line_sensor',
        )
    ])
