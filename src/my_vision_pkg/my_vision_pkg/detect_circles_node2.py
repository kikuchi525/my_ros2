#!/home/tabatakenta/.pyenv/versions/3.10.12/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import Float32MultiArray
import math

class CirclePublisherNode(Node):
    def __init__(self):
        super().__init__('circle_publisher_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/circle_data', 10)

        # RealSenseの初期化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.get_logger().info('✅ CirclePublisherNode started.')

        # タイマーで定期実行（約30fps）
        self.timer = self.create_timer(0.033, self.detect_and_publish)

    def detect_and_publish(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2, 2)

        # 円検出
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
            param1=150, param2=80, minRadius=0, maxRadius=0)

        data = Float32MultiArray()

        if circles is not None:
            circles = np.uint16(np.around(circles))
            closest_circle = None
            min_dist = float('inf')

            for i in circles[0, :]:
                x, y, r = float(i[0]), float(i[1]), float(i[2])
                dx = x - 640
                dy = y - 360
                dist = math.hypot(dx, dy)

                # 一番中心に近い円を選ぶ
                if dist < min_dist:
                    min_dist = dist
                    closest_circle = (x, y, r)

                # 画像上に全て描画
                cv2.circle(img, (int(x), int(y)), int(r), (0, 255, 0), 2)
                cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), 3)
                cv2.putText(img, f"({int(x)}, {int(y)}, r={int(r)})",
                            (int(x)+10, int(y)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # 最も中心に近い円をパブリッシュ
            if closest_circle:
                x, y, r = closest_circle
                self.get_logger().info(f'🎯 Closest circle: x={x:.1f}, y={y:.1f}, r={r:.1f}')
                data.data.extend([x, y, r])
                self.publisher_.publish(data)
        else:
            self.get_logger().info('🟡 No circles detected.')
            data.data = []  # 空データを明示的に送信
            self.publisher_.publish(data)

        # 表示
        cv2.imshow('Circle Detection', img)
        key = cv2.waitKey(1)
        if key == 27:  # ESCキーで終了
            self.get_logger().info('ESC pressed. Exiting...')
            rclpy.shutdown()

    def cleanup(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
