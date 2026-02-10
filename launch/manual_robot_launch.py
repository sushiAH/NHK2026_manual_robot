from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    publish_twist_node = Node(
        package="manual_robot",
        executable="publish_twist_node",
    )

    subscribe_twist_node = Node(
        package="manual_robot",
        executable="subscribe_twist_node",
    )

    control_spear_node = Node(
        package="manual_robot",
        executable="control_spear_node",
    )

    control_box_node = Node(
        package="manual_robot",
        executable="control_box_node",
    )

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    dyna_handler_node = Node(
        package="ah_ros2_dynamixel",
        executable="dyna_handler_node",
    )

    ld.add_action(publish_twist_node)
    ld.add_action(subscribe_twist_node)
    ld.add_action(control_box_node)
    ld.add_action(control_spear_node)
    ld.add_action(joy_linux_node)
    ld.add_action(dyna_handler_node)

    return ld
