"""twistをsubscribeして、esp32にモーター指令値を送信する"""

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


def from_twist_to_motor_vel(vx, vy, w, L, fy):
    V_1 = -(-vy + vx + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_2 = (-vy - vx + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_3 = (vy + vx + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_4 = (vy - vx + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)

    return (V_1, V_2, V_3, V_4)


class twist_subscriber(Node):
    def __init__(self):
        super().__init__("twist_subscriber")

        # シリアル立ち上げ
        self.ser = serial.Serial("/dev/ttyUSB0", 921600, timeout=0)

        # 動作モードの送信
        send_to_4motor(0, 2, 2, 2, 2, self.ser)

        # ゲインの送信
        # P_gain
        send_to_4motor(9, 100, 100, 100, 100, self.ser)
        # I_gain
        send_to_4motor(10, 10000, 10000, 10000, 10000, self.ser)
        # D_gain
        send_to_4motor(11, 0, 0, 0, 0, self.ser)

        self.subscription_twist_joy = self.create_subscription(
            Twist,  # メッセージの型
            "/cmd_vel_joy",  # 購読するトピック名
            self.twist_by_joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_twist_joy

        # --- Config ---
        # 車体横の長さ
        self.L = 0.3
        # 車体中心からタイヤまでの距離
        self.fy = 0.127

        # メンバーの初期化
        self.joy_linear_x = 0
        self.joy_linear_y = 0
        self.joy_w = 0

        # wirte_to_motorの割り込み設定
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.write_to_motor)

    def twist_by_joy_callback(self, msg):
        """subscribe twist message, store twist in member value

        Args:
            msg (Twist): [twist message]
        """
        self.joy_linear_x = msg.linear.x
        self.joy_linear_y = msg.linear.y
        self.joy_w = msg.angular.z

    def write_to_motor(self):
        """Twistをメカナムホイール逆運動学で、各モーターの速度指令値に分解。4つの速度指令値を一つのパケットでesp32に送信する"""
        vx = self.joy_linear_x
        vy = self.joy_linear_y
        w = self.joy_w

        V_1, V_2, V_3, V_4 = from_twist_to_motor_vel(vx, vy, w, self.L, self.fy)

        send_4value_by_one_packet(2, V_1, V_2, V_3, V_4, self.ser)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = twist_subscriber()

    rclpy.spin(twist_subscriber_node)  # ノードをスピンさせる
    twist_subscriber_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
