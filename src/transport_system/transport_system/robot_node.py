import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from transport_system.action import MoveToPosition  # Action personnalisée
from rclpy.qos import QoSProfile
import time

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self._action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to_position',
            self.execute_callback
        )
        self.get_logger().info('[ROBOT] Action server ready.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'[ROBOT] Received goal: {goal_handle.request.target_position}')

        feedback_msg = MoveToPosition.Feedback()
        for i in range(5):
            feedback_msg.current_status = f"Moving... {i*20}%"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.get_logger().info('[ROBOT] Arrived at target position.')
        goal_handle.succeed()

        result = MoveToPosition.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
