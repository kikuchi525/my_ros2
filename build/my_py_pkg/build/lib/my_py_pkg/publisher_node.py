# float_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class FloatPublisher(Node):
    def __init__(self):
        super().__init__('float_publisher')
        self.publisher_ = self.create_publisher(Float64, 'numbers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.value = 1.0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.value += 1.0

def main(args=None):
    rclpy.init(args=args)
    node = FloatPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
