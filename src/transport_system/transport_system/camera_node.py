import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(String, 'detected_object', 10)
        self.timer = self.create_timer(2.0, self.publish_object)
        self.objects = ['obj_001', 'obj_002', 'obj_003', 'obj_004']

    def publish_object(self):
        obj_id = random.choice(self.objects)
        msg = String()
        msg.data = obj_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'[CAMERA] Detected: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
