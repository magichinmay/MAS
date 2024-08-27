import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mas_cpp.action import Motor
import RPi.GPIO as GPIO
import time

class MotorControlServer(Node):
    def __init__(self):
        super().__init__('motor_control_server')

        # GPIO Setup
        self.motor_pin = 18  # Example GPIO pin for motor control
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.motor_pin, 1000)  # 1 kHz PWM frequency
        self.pwm.start(0)

        # Action Server Setup
        self._action_server = ActionServer(
            self,
            Motor,
            'Motor',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Get goal parameters
        speed = goal_handle.request.speed
        duration = goal_handle.request.duration

        success = True

        feedback_msg = Motor.Feedback()
        feedback_msg.current_speed = 0.0

        start_time = time.time()
        self.pwm.ChangeDutyCycle(speed)  # Set speed

        while time.time() - start_time < duration:
            feedback_msg.current_speed = speed
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Motor running at {speed} for {time.time() - start_time} seconds')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.pwm.ChangeDutyCycle(0)  # Stop motor
        GPIO.cleanup()  # Cleanup GPIO

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
