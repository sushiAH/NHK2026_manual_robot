from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    node1 = Node(
        package="manual_robot",  # package_name
        name="publish_twist",  # node_name
        executable="twist_publisher",  # file_name
    )

    node2 = Node(
        package="manual_robot",
        name="subscribe_twist_can",
        executable="twist_subscriber_can",
    )

    node3 = Node(
        package="manual_robot",
        name="arm_hand_controller",
        executable="arm_hand_controller",
    )

    node4 = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)

    return ld
