import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Exemple simple

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.srv = self.create_service(AddTwoInts, 'pickup_object', self.pickup_callback)

    def pickup_callback(self, request, response):
        self.get_logger().info(f'[ARM] Picking up object ID: {request.a}, force: {request.b}')
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
