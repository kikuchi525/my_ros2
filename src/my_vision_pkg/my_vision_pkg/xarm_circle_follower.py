#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xarm.wrapper import XArmAPI
import time

class XArmCircleFollower(Node):
    def __init__(self):
        super().__init__('xarm_circle_follower')

        # xArm 接続
        self.arm = XArmAPI('192.168.1.214')  # ← あなたのxArmのIPアドレスに変更
        self.arm.motion_enable(True)
        self.arm.set_mode(0)  # position mode
        self.arm.set_state(0)
        self.arm.set_position(reset=True, wait=True)  # 初期化（必要に応じて）

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/circle_data',
            self.listener_callback,
            10)

        self.get_logger().info('🟢 XArmCircleFollower started.')

        self.margin = 20  # 画像中心からのしきい値（ピクセル）
        self.move_step = 5.0  # 移動距離（mm）

    def listener_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn('❌ circle_data に不正な長さのデータ')
            return

        x, y, r = msg.data
        dx = x - 640  # 画像中心x
        dy = y - 360  # 画像中心y

        move_x = 0.0
        move_y = 0.0

        if dx > self.margin:
            move_x = +self.move_step
        elif dx < -self.margin:
            move_x = -self.move_step

        if dy < -self.margin:
            move_y = +self.move_step
        elif dy > self.margin:
            move_y = -self.move_step

        if move_x == 0.0 and move_y == 0.0:
            self.get_logger().info('🟡 円は中心付近。アームは動かしません。')
            return

        self.get_logger().info(f'➡ アーム移動: x={move_x}mm, y={move_y}mm')

        # 相対移動
        self.arm.set_position(x=move_x, y=move_y, relative=True, wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = XArmCircleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

