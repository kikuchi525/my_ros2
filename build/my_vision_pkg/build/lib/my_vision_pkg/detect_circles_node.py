#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

import cv2
import numpy as np
import pyrealsense2 as rs

class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')

        # RealSense初期化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 電流パブリッシャ
        self.current_pub = self.create_publisher(Int16, 'goal_current', 10)

        # パラメータ（必要に応じて調整）
        self.param1 = 150
        self.param2 = 80
        self.current_on = 200   # 円検出中（+20.0mA）
        self.current_off = -200 # 非検出中（-20.0mA）

        # タイマー（約30fps）
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info('✅ Circle detector (auto current) started.')

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn("No color frame received.")
            return

        img = np.asanyarray(color_frame.get_data())
        circle_found = self.detect_circles(img)

        # 電流パブリッシュ
        msg = Int16()
        msg.data = self.current_on if circle_found else self.current_off
        self.current_pub.publish(msg)

        # ウィンドウ表示
        cv2.imshow('Circle Detection', img)
        cv2.waitKey(1)

    def detect_circles(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                   param1=self.param1, param2=self.param2,
                                   minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
            return True
        return False

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
