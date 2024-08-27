import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mas_cpp.action import Motor

class MotorControlClient(Node):

    def __init__(self):
        super().__init__('motor_control_client')
        self._action_client = ActionClient(self, Motor, 'Motor')

    def send_goal(self, speed, duration):
        goal_msg = Motor.Goal()
        goal_msg.speed = speed
        goal_msg.duration = duration

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.current_speed}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Success: {result.success}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    motor_control_client = MotorControlClient()
    motor_control_client.send_goal(speed=1.0, duration=5.0)
    rclpy.spin(motor_control_client)

if __name__ == '__main__':
    main()
