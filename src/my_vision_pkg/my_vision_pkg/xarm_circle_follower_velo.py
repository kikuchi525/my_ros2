#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xarm.wrapper import XArmAPI

class XArmCircleVelocityController(Node):
    def __init__(self):
        super().__init__('xarm_circle_velocity_controller')

        # xArm IPアドレスに置き換えてください
        self.arm = XArmAPI('192.168.1.214')

        # 速度制御モードに設定
        self.arm.motion_enable(True)
        self.arm.set_mode(5)  # 速度制御モード
        self.arm.set_state(0) # ON状態

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/circle_data',
            self.listener_callback,
            10)

        self.get_logger().info('🟢 xArm Velocity Controller started.')

        self.margin = 20      # 中心からのズレ許容ピクセル
        self.speed_value = 100  # mm/s の速度値

    def listener_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn('🛑 円が検出されていないので停止します')
            self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
            return

        x, y, r = msg.data
        dx = x - 640  # 画像中心X
        dy = y - 360  # 画像中心Y

        vx = 0
        vy = 0

        if dx > self.margin:
            vx = +self.speed_value
        elif dx < -self.margin:
            vx = -self.speed_value

        if dy < -self.margin:
            vy = +self.speed_value
        elif dy > self.margin:
            vy = -self.speed_value

        # 速度指令のベクトル [vx, vy, vz, rx, ry, rz]
        velocity_command = [vx, vy, 0, 0, 0, 0]

        self.get_logger().info(f'速度指令: vx={vx}, vy={vy}')

        self.arm.vc_set_cartesian_velocity(velocity_command)

    def destroy_node(self):
        # 停止指令を送る
        self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
        self.arm.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = XArmCircleVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

