from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    publish_twist_node = Node(package="manual_robot",
                              executable="publish_twist_node",
                              output="screen")

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

    publish_feedback_node = Node(package="manual_robot",
                                 executable="publish_feedback_node")

    dyna_handler_node = Node(package="ah_ros2_dynamixel",
                             executable="dyna_handler_node",
                             parameters=[{
                                 "port_name": "/dev/ttyUSB0",
                             }])

    rosbridge_node = Node(package="rosbridge_server",
                          executable="rosbridge_websocket",
                          name="rosbridge_websocket",
                          output="screen",
                          parameters=[{
                              "port": 9090,
                              "address:": "",
                              "retry_startup_delay": 5.0,
                              "fragment_timeout": 600,
                              "delay_between_messages": 0.0,
                              "unregister_timeout": 10.0,
                          }])

    ld.add_action(publish_twist_node)
    ld.add_action(subscribe_twist_node)
    ld.add_action(control_box_node)
    ld.add_action(control_spear_node)
    ld.add_action(joy_linux_node)
    ld.add_action(dyna_handler_node)
    ld.add_action(publish_feedback_node)
    ld.add_action(rosbridge_node)

    return ld
