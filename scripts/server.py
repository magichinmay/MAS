import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mas_cpp.action import Motor

class MotorControlServer(Node):

    def __init__(self):
        super().__init__('motor_control_server')
        self._action_server = ActionServer(
            self,
            Motor,
            'Motor',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Example: Run motor at the specified speed for the specified duration
        speed = goal_handle.request.speed
        duration = goal_handle.request.duration
        success = True

        feedback_msg = Motor.Feedback()
        feedback_msg.current_speed = 0.0  # Assume starting from 0 speed

        for i in range(int(duration * 10)):  # Simulate motor running
            feedback_msg.current_speed = speed
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Motor running at {speed} for {i/10.0} seconds')
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()

        result = Motor.Result()
        result.success = success
        return result

def main(args=None):
    rclpy.init(args=args)
    motor_control_server = MotorControlServer()
    rclpy.spin(motor_control_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
