#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorTestNode(Node):
    """
    Physical Verification Node for 4-Wheel Differential Drive Robot.
    Sequentially tests FORWARD, REVERSE, LEFT, RIGHT, and STOP via /cmd_vel.
    """

    def __init__(self):
        super().__init__('motor_test')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('========================================================')
        self.get_logger().info('   PHYSICAL MOTOR TEST INITIALIZED (Hour 1 Verification)')
        self.get_logger().info('   Sequence: FORWARD -> REVERSE -> LEFT -> RIGHT -> STOP')
        self.get_logger().info('========================================================')

    def publish_twist(self, linear_x: float, angular_z: float, duration_sec: float, label: str):
        self.get_logger().info(f'>>> EXECUTING TEST: [{label}] (linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s) for {duration_sec}s')
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        # Stream cmd_vel at 20 Hz for the given duration
        rate = 0.05
        steps = int(duration_sec / rate)
        for _ in range(steps):
            self.pub_cmd_vel.publish(msg)
            time.sleep(rate)

    def publish_stop(self, duration_sec: float = 1.0):
        self.get_logger().info(f'>>> MOTOR STOP: Halting motors for {duration_sec}s')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        rate = 0.05
        steps = int(duration_sec / rate)
        for _ in range(steps):
            self.pub_cmd_vel.publish(msg)
            time.sleep(rate)

    def run_test_sequence(self):
        time.sleep(1.0)  # Settle time

        # 1. FORWARD
        self.publish_twist(linear_x=0.20, angular_z=0.0, duration_sec=2.0, label='FORWARD (Both sides spin forward)')
        self.publish_stop(duration_sec=1.0)

        # 2. REVERSE
        self.publish_twist(linear_x=-0.20, angular_z=0.0, duration_sec=2.0, label='REVERSE (Both sides spin backward)')
        self.publish_stop(duration_sec=1.0)

        # 3. LEFT (Counter-Clockwise Rotation)
        self.publish_twist(linear_x=0.0, angular_z=1.0, duration_sec=2.0, label='LEFT (Left wheels backward, Right wheels forward)')
        self.publish_stop(duration_sec=1.0)

        # 4. RIGHT (Clockwise Rotation)
        self.publish_twist(linear_x=0.0, angular_z=-1.0, duration_sec=2.0, label='RIGHT (Left wheels forward, Right wheels backward)')
        self.publish_stop(duration_sec=1.0)

        self.get_logger().info('========================================================')
        self.get_logger().info('   MOTOR TEST SEQUENCE COMPLETED SUCCESSFULLY.')
        self.get_logger().info('========================================================')


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        node.run_test_sequence()
    except KeyboardInterrupt:
        node.publish_stop(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
