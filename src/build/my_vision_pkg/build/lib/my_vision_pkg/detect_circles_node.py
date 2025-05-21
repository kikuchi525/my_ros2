#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os
from datetime import datetime

class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')
        self.get_logger().info('Circle detector started.')

        # リアルセンスの初期化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 保存フォルダ作成
        self.save_folder = 'pic'
        os.makedirs(self.save_folder, exist_ok=True)

        # パラメータ
        self.param1 = 150
        self.param2 = 80

        self.detect_and_save()
        self.pipeline.stop()
        self.get_logger().info('Detection finished.')

    def detect_and_save(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().error('No color frame received.')
            return

        img = np.asanyarray(color_frame.get_data())
        self.detect_circles(img, self.param1, self.param2)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_folder, f'captured_{timestamp}.png')
        cv2.imwrite(filename, img)
        self.get_logger().info(f'Saved: {filename}')

    def detect_circles(self, img, param1, param2):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                   param1=param1, param2=param2, minRadius=0, maxRadius=0)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
