from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rope_direction_publisher_node = Node(
        package='cavepi_controller',
        executable='rope_pose_waypoints_pub',
        name='rope_pose_waypoints_pub',
        output='screen'
    )

    pixhawk_node = Node(
        package='cavepi_controller',
        executable='test_pixhawk',
        name='test_pixhawk',
        output='screen'
    )

    autopliot_node = Node(
        package='cavepi_controller',
        executable='autopilot',
        name='autopilot',
        output='screen'
    )

    ld.add_action(rope_direction_publisher_node)
    ld.add_action(pixhawk_node)
    ld.add_action(autopliot_node)

    return ld