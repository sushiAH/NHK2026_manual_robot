"""joystickをsubscribeして、twistをpublishする"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from manual_robot.lib.ah_uart import *


class twist_publisher(Node):
    """subscribe joystick and publish twist message

    Attributes:
        subscription_joy: instance of subscribe
        twist_publisher:  instance of publish
    """

    def __init__(self):
        super().__init__("twist_publisher")

        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_joy

        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)

    def joy_callback(self, msg):

        axes_values = msg.axes

        twist = Twist()

        if abs(axes_values[1]) < 0.1:
            axes_values[1] = 0
        if abs(axes_values[0]) < 0.1:
            axes_values[0] = 0
        if abs(axes_values[3]) < 0.1:
            axes_values[3] = 0

        twist.linear.x = axes_values[1] * 2
        twist.linear.y = axes_values[0] * 2
        twist.angular.z = axes_values[3] * 1

        self.twist_publisher.publish(twist)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_publisher_node = twist_publisher()

    rclpy.spin(twist_publisher_node)  # ノードをスピンさせる
    twist_publisher_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
