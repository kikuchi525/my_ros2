# circle_subscriber_direction.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CircleDirectionSubscriber(Node):
    def __init__(self):
        super().__init__('circle_direction_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/circle_data',
            self.listener_callback,
            10)
        self.get_logger().info('Direction-aware circle subscriber started.')

    def listener_callback(self, msg):
        data = msg.data
        num_circles = len(data) // 3

        for i in range(num_circles):
            x = data[i * 3]
            y = data[i * 3 + 1]
            r = data[i * 3 + 2]

            dx = x - 640  # 画像中心 x
            dy = y - 360  # 画像中心 y

            direction = ""

            # 水平方向
            if dx > 20:
                direction += "右"
            elif dx < -20:
                direction += "左"
            else:
                direction += "中央"

            # 垂直方向
            if dy > 20:
                direction += "下"
            elif dy < -20:
                direction += "上"
            else:
                direction += "中央"

            self.get_logger().info(
                f"円の中心 (x={x:.1f}, y={y:.1f}, r={r:.1f}) は画像中心から見て → 【{direction}】")

def main(args=None):
    rclpy.init(args=args)
    node = CircleDirectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
