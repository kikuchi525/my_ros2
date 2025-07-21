# float_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class FloatSubscriber(Node):
    def __init__(self):
        super().__init__('float_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'numbers',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        original = msg.data
        modified = original * 2
        self.get_logger().info(f'Received: {original}, Doubled: {modified}')

def main(args=None):
    rclpy.init(args=args)
    node = FloatSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
