#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Point
from my_vision_pkg.msg import Circle, Circles  # ← カスタムメッセージをimport


class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')
        self.get_logger().info('Circle detector with center + radius publisher started.')

        # RealSense初期化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # パブリッシャー作成（カスタムメッセージCirclesを使う）
        self.publisher_ = self.create_publisher(Circles, '/circle_data', 10)

        # Hough円検出パラメータ
        self.param1 = 150
        self.param2 = 80

        # タイマー設定（約30fps）
        self.timer = self.create_timer(0.033, self.process_frame)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn('No color frame received.')
            return

        img = np.asanyarray(color_frame.get_data())
        self.detect_circles(img, self.param1, self.param2)

        # 表示
        cv2.imshow('Circle Detection', img)
        key = cv2.waitKey(1)
        if key == 27:  # ESCで終了
            self.get_logger().info('ESC pressed. Shutting down...')
            rclpy.shutdown()

    def detect_circles(self, img, param1, param2):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                   param1=param1, param2=param2, minRadius=0, maxRadius=0)

        msg = Circles()

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                x, y, r = i[0], i[1], i[2]

                circle_msg = Circle()
                circle_msg.center = Point(x=float(x), y=float(y), z=0.0)
                circle_msg.radius = float(r)
                msg.circles.append(circle_msg)

                # 円描画
                cv2.circle(img, (x, y), r, (0, 255, 0), 2)
                cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

                # 座標表示
                coord_text = f'({x}, {y}, r={r})'
                cv2.putText(img, coord_text, (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                self.get_logger().info(f'Detected circle: x={x}, y={y}, r={r}')
            
            # すべての円をまとめてパブリッシュ
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('No circles detected.')


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
