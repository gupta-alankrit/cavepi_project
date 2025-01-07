from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cavepi_detection',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cavepi_detection',
                    executable='qr_detector',
                    name='qr_detector',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='cavepi_detection',
                    executable='caveline_detector',
                    name='caveline_detector',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=6.0, 
            actions=[
                Node(
                    package='cavepi_detection',
                    executable='data_sender',
                    name='data_sender',
                    output='screen'
                )
            ]
        )
    ])

