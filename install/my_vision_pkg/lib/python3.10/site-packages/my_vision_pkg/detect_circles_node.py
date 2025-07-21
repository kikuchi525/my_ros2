#!/home/tabatakenta/.pyenv/versions/3.10.12/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Point


class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')
        self.get_logger().info('Circle detector with center publisher started.')

        # RealSense初期化
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # パブリッシャー作成
        self.publisher_ = self.create_publisher(Point, '/circle_center', 10)

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

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                x, y, r = i[0], i[1], i[2]
                # 円描画
                cv2.circle(img, (x, y), r, (0, 255, 0), 2)
                cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

                # 座標表示
                coord_text = f'({x}, {y})'
                cv2.putText(img, coord_text, (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # ログ出力
                self.get_logger().info(f'Detected circle center at: x={x}, y={y}')

                # パブリッシュ
                point_msg = Point()
                point_msg.x = float(x)
                point_msg.y = float(y)
                point_msg.z = 0.0
                self.publisher_.publish(point_msg)
                break  # 最初の円だけ送信（必要なら複数も対応可能）
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
