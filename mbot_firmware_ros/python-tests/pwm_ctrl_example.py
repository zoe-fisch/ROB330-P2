import rclpy
from rclpy.node import Node
import time
from mbot_interfaces.msg import MotorPWM


class PWMPublisher(Node):

    def __init__(self):
        super().__init__('pwm_publisher')
        self.publisher_ = self.create_publisher(MotorPWM, 'motor_pwm_cmd', 10)
        # Publish commands at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MotorPWM()
        msg.pwm[0] = 0.25
        msg.pwm[1] = 1.0
        msg.pwm[2] = 0.0
        self.publisher_.publish(msg)
        # Log at debug level to avoid spamming the console
        self.get_logger().debug(f'Publishing PWM: {list(msg.pwm)}')


def main(args=None):
    rclpy.init(args=args)
    pwm_publisher = PWMPublisher()

    start_time = time.time()
    try:
        while rclpy.ok() and (time.time() - start_time) < 5.0:
            # Spin the executor; wake up at most every 0.1 s to keep latency low
            rclpy.spin_once(pwm_publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        pwm_publisher.get_logger().info('Interrupted by user')

    # Send a zero-PWM command to stop the motors before shutting down
    stop_msg = MotorPWM()
    stop_msg.pwm[0] = 0.0
    stop_msg.pwm[1] = 0.0
    stop_msg.pwm[2] = 0.0
    pwm_publisher.publisher_.publish(stop_msg)

    pwm_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()