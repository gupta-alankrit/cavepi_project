from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pixhawk_node = Node(
        package='cavepi_controller',
        executable='pixhawk_logic',
        name='pixhawk_logic',
        output='screen'
    )

    autopliot_node = Node(
        package='cavepi_controller',
        executable='autopilot',
        name='autopilot',
        output='screen'
    )

    ld.add_action(pixhawk_node)
    ld.add_action(autopliot_node)

    return ld
