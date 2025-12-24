"""やり回収、ボックス回収機構を制御する。"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import numpy as np

import atexit
from sensor_msgs.msg import Joy

from manual_robot.lib.ah_python_can import *
from manual_robot.lib.dyna_lib import dxl_controller


bus = can.interface.Bus(
    bustype="socketcan", channel="can0", asynchronous=True, bitrate=1000000
)


class SpearController:
    def __init__(self):

        # init_dynamixel
        self.dxl_1 = dxl_controller("/dev/ttyUSB0", 0, 3)
        self.dxl_2 = dxl_controller("/dev/ttyUSB0", 1, 3)
        self.dxl_3 = dxl_controller("/dev/ttyUSB0", 2, 3)

        # air
        send_packet_1byte(0x021, 0, 5, bus)  # set_operating

        self.state_counter = 0
        self.last_button_state = 0
        self.now_button_state = 0

        # 取る状態のパターン
        self.state_length = 4

    def update_spear_state(self):
        """やり回収機構の状態を更新"""
        if self.now_button_state == 1 and self.last_button_state == 0:
            self.state_counter += 1

        self.state_counter = self.state_counter % self.state_length

        self.last_button_state = self.now_button_state

    def move_joint(self):
        """state_counterに従って動作"""
        if self.state_counter == 0:
            self.dxl_1.add_sync_param_pos(3100)  # 横
            self.dxl_2.add_sync_param_pos(3300)  # 開ける
            self.dxl_3.add_sync_param_pos(2800)  # 開ける

            send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる

        elif self.state_counter == 1:
            send_packet_1byte(0x021, 12, 1, bus)  # air 開く
            self.dxl_1.add_sync_param_pos(3300)  # 縦

        elif self.state_counter == 2:
            self.dxl_2.add_sync_param_pos(3100)  # 閉じる
            self.dxl_3.add_sync_param_pos(3100)  # 閉じる

        elif self.state_counter == 3:
            self.dxl_1.add_sync_param_pos(3100)  # 横
            send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる


class BoxController:
    def __init__(self):
        # init_dynamixel
        self.dxl_4 = dxl_controller("/dev/ttyUSB0", 3, 3)
        self.dxl_5 = dxl_controller("/dev/ttyUSB0", 4, 3)

        # box hand up down
        send_packet_1byte(0x020, 0, 2, bus)  # set_operating
        send_packet_4byte(0x020, 6, 10, bus)  # set pos_p_gain

        self.state_counter = 0
        self.last_button_state = 0
        self.now_button_state = 0

        # 取る状態のパターン
        self.state_length = 3

    def update_box_state(self):
        """ボックス回収機構の状態を更新"""
        if self.now_button_state == 1 and self.last_button_state == 0:
            self.state_counter += 1

        self.state_counter = self.state_counter % self.state_length

        self.last_button_state = self.now_button_state

    def move_joint(self):
        """state_counterに従って動作"""
        if self.state_counter == 0:
            self.dxl_4.add_sync_param_pos(1500)  # 縮む
            self.dxl_5.add_sync_param_pos(2100)  # ハンド開く
            send_packet_4byte(0x020, 1, 3000, bus)  # 昇降　下

        elif self.state_counter == 1:
            self.dxl_4.add_sync_param_pos(3000)  # 伸び
            send_packet_4byte(0x020, 1, 2000, bus)  # 昇降　上

        elif self.state_counter == 2:
            self.dxl_5.add_sync_param_pos(1400)  # ハンド閉じる

        # 全データを一つのパケットにまとめて送信
        self.dxl_5.write_group_dyna_pos()


class box_spear_controller(Node):
    def __init__(self):
        super().__init__("box_spear_controller")

        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_joy

        self.spear_controller = SpearController()
        self.box_controller = BoxController()

    def joy_callback(self, msg):
        """joyを受取、各機構を動作

        Args:
            msg (Joy): joy_stick_message
        """
        self.spear_controller.now_button_state = msg.buttons[0]
        self.box_controller.now_button_state = msg.buttons[1]

        self.spear_controller.update_spear_state()
        self.spear_controller.move_joint()

        self.box_controller.update_box_state()
        self.box_controller.move_joint()


def main():
    rclpy.init()  # rclpyライブラリの初期化

    box_spear_controller_node = box_spear_controller()

    rclpy.spin(box_spear_controller_node)  # ノードをスピンさせる
    box_spear_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
