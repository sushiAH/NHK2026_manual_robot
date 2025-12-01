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
        name="subscribe_twist",
        executable="twist_subscriber",
    )

    ld.add_action(node1)

    return ld
