from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cavepi_detection',
            executable='test_camera',
            name='test_camera',
            output='screen'
        ),

        Node(
            package='cavepi_detection',
            executable='qr_detector',
            name='qr_detector',
            output='screen'
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
