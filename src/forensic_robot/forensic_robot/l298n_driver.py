#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Attempt to import GPIO libraries gracefully
HAS_GPIO = False
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except (ImportError, RuntimeError):
    try:
        import lgpio
        HAS_GPIO = True
    except (ImportError, RuntimeError):
        pass


class L298NMotorDriver(Node):
    """
    ROS 2 Node for controlling 4 DC motors via L298N Dual H-Bridge Driver
    on a Raspberry Pi 4 for a 4-wheel differential-drive forensic robot.
    """

    def __init__(self):
        super().__init__('l298n_driver')

        # -------------------------------------------------------------
        # 1. Parameter Declarations (Pinout & Kinematics)
        # -------------------------------------------------------------
        # Left Motor Pins (IN1, IN2, ENA - PWM)
        self.declare_parameter('in1_pin', 24)
        self.declare_parameter('in2_pin', 23)
        self.declare_parameter('ena_pin', 12)  # Hardware PWM channel 0 (BCM 12)

        # Right Motor Pins (IN3, IN4, ENB - PWM)
        self.declare_parameter('in3_pin', 22)
        self.declare_parameter('in4_pin', 27)
        self.declare_parameter('enb_pin', 13)  # Hardware PWM channel 1 (BCM 13)

        # Kinematics & Motor Limits
        self.declare_parameter('wheel_radius', 0.0325)  # 65 mm diameter = 0.0325 m radius
        self.declare_parameter('wheel_base', 0.200)     # 200 mm track width
        self.declare_parameter('max_rpm', 150.0)        # Rated DC motor RPM at nominal voltage
        self.declare_parameter('pwm_frequency', 1000)   # PWM frequency in Hz
        self.declare_parameter('min_pwm_duty', 25.0)    # Deadband threshold to overcome gearbox friction
        self.declare_parameter('timeout_sec', 0.5)      # Safety watchdog timeout

        # Retrieve parameter values
        self.in1 = self.get_parameter('in1_pin').value
        self.in2 = self.get_parameter('in2_pin').value
        self.ena = self.get_parameter('ena_pin').value

        self.in3 = self.get_parameter('in3_pin').value
        self.in4 = self.get_parameter('in4_pin').value
        self.enb = self.get_parameter('enb_pin').value

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.max_rad_s = (self.max_rpm * 2.0 * 3.14159265) / 60.0  # Max wheel angular speed
        self.min_pwm = float(self.get_parameter('min_pwm_duty').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.pwm_freq = int(self.get_parameter('pwm_frequency').value)

        # -------------------------------------------------------------
        # 2. Hardware / GPIO Initialization
        # -------------------------------------------------------------
        self.gpio_available = HAS_GPIO
        self.pwm_left = None
        self.pwm_right = None

        if self.gpio_available:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                # Set pin modes
                GPIO.setup(self.in1, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.in2, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.ena, GPIO.OUT, initial=GPIO.LOW)

                GPIO.setup(self.in3, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.in4, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.enb, GPIO.OUT, initial=GPIO.LOW)

                # Setup PWM
                self.pwm_left = GPIO.PWM(self.ena, self.pwm_freq)
                self.pwm_right = GPIO.PWM(self.enb, self.pwm_freq)
                self.pwm_left.start(0)
                self.pwm_right.start(0)

                self.get_logger().info('L298N GPIO hardware initialized successfully on RPi.')
            except Exception as e:
                self.gpio_available = False
                self.get_logger().warn(f'Failed to initialize GPIO: {e}. Running in SIMULATION/MOCK mode.')
        else:
            self.get_logger().warn('No GPIO library found. Running L298N driver in SIMULATION/MOCK mode.')

        # -------------------------------------------------------------
        # 3. Subscriptions & Watchdog
        # -------------------------------------------------------------
        self.last_cmd_time = time.time()
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 20 Hz Watchdog Timer for safety stop
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)
        self.get_logger().info('L298N Motor Driver Node is active and listening on /cmd_vel.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        v = msg.linear.x
        omega = msg.angular.z

        # Differential drive inverse kinematics
        # Left and Right linear velocities (m/s)
        v_left = v - (omega * self.L / 2.0)
        v_right = v + (omega * self.L / 2.0)

        # Angular velocities (rad/s)
        w_left = v_left / self.r
        w_right = v_right / self.r

        self.set_motor_speeds(w_left, w_right)

    def set_motor_speeds(self, w_left: float, w_right: float):
        """
        Maps desired angular wheel velocities to L298N direction & PWM duty cycle.
        """
        duty_left, dir_left = self._calc_duty_and_dir(w_left)
        duty_right, dir_right = self._calc_duty_and_dir(w_right)

        if self.gpio_available:
            # Set Left Motors (IN1 / IN2)
            if dir_left == 1:       # Forward
                GPIO.output(self.in1, GPIO.HIGH)
                GPIO.output(self.in2, GPIO.LOW)
            elif dir_left == -1:    # Reverse
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.HIGH)
            else:                   # Stop
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(duty_left)

            # Set Right Motors (IN3 / IN4)
            if dir_right == 1:      # Forward
                GPIO.output(self.in3, GPIO.HIGH)
                GPIO.output(self.in4, GPIO.LOW)
            elif dir_right == -1:   # Reverse
                GPIO.output(self.in3, GPIO.LOW)
                GPIO.output(self.in4, GPIO.HIGH)
            else:                   # Stop
                GPIO.output(self.in3, GPIO.LOW)
                GPIO.output(self.in4, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(duty_right)
        else:
            # Simulated telemetry log for testing off-board
            self.get_logger().debug(
                f'[MOCK MOTOR] Left: {dir_left:+d} @ {duty_left:5.1f}% | Right: {dir_right:+d} @ {duty_right:5.1f}%'
            )

    def _calc_duty_and_dir(self, w: float):
        """Calculates direction (1, -1, 0) and PWM duty cycle (0.0 to 100.0%)."""
        if abs(w) < 1e-4:
            return 0.0, 0

        direction = 1 if w > 0 else -1
        # Normalize to maximum motor RPM
        ratio = min(abs(w) / self.max_rad_s, 1.0)
        duty = self.min_pwm + (ratio * (100.0 - self.min_pwm))
        duty = max(0.0, min(100.0, duty))
        return duty, direction

    def stop_motors(self):
        """Halts all 4 motors immediately."""
        if self.gpio_available:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.LOW)
            if self.pwm_left:
                self.pwm_left.ChangeDutyCycle(0)
            if self.pwm_right:
                self.pwm_right.ChangeDutyCycle(0)

    def watchdog_callback(self):
        """Safety timeout stops motors if cmd_vel stream is interrupted."""
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self.stop_motors()

    def destroy_node(self):
        self.stop_motors()
        if self.gpio_available:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L298NMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
