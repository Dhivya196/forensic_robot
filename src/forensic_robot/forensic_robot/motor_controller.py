import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorController(Node):
    """
    Simulation & Kinematics Testing Node (Hardware-Independent).
    Listens to /cmd_vel, calculates differential-drive wheel angular velocities,
    and logs kinematic telemetry for testing without accessing physical GPIO.
    
    NOTE: For physical hardware motor control on Raspberry Pi + L298N,
          use the dedicated node: forensic_robot.l298n_driver.
    """
    def __init__(self):
        super().__init__('motor_controller')

        # 65 mm wheel -> radius 0.0325 m
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.20)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info(
            'Simulation Motor Controller ready (Kinematics Logger). GPIO output is disabled.'
        )

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # Differential-drive equations
        v_left = v - omega * self.L / 2.0
        v_right = v + omega * self.L / 2.0

        # Linear wheel velocity -> angular wheel velocity
        w_left = v_left / self.r
        w_right = v_right / self.r

        self.get_logger().info(
            f'left={w_left:.2f} rad/s | right={w_right:.2f} rad/s'
        )

        # TODO:
        # Replace this with the GPIO/PWM code for the FINAL motor driver.
        # Do not connect motors directly to Raspberry Pi GPIO.


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
