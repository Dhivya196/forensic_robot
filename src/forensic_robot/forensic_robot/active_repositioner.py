#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ActiveRepositioner(Node):
    """
    ROS 2 Active Repositioning Node (Physical Prototype Baseline).
    
    WORKFLOW:
    1. Listens on /evidence_candidates_uncertain for intermediate-confidence detections.
    2. Executes a direct, controlled viewpoint rotation (~35 degrees) via /cmd_vel.
    3. Issues an explicit stop command to halt the robot.
    4. Triggers secondary follow-up image capture on /capture_evidence.
    """

    def __init__(self):
        super().__init__('active_repositioner')

        # -------------------------------------------------------------
        # 1. Parameters
        # -------------------------------------------------------------
        self.declare_parameter('reposition_angle_deg', 35.0)     # Target rotation angle in degrees
        self.declare_parameter('angular_speed_rad_s', 0.40)      # Commanded rotation speed
        self.declare_parameter('reposition_cooldown_sec', 8.0)   # Minimum delay between reposition maneuvers
        self.declare_parameter('settle_time_sec', 0.8)           # Wait time after motion stop before capture

        self.angle_deg = float(self.get_parameter('reposition_angle_deg').value)
        self.angular_speed = float(self.get_parameter('angular_speed_rad_s').value)
        self.cooldown_sec = float(self.get_parameter('reposition_cooldown_sec').value)
        self.settle_time = float(self.get_parameter('settle_time_sec').value)

        # State tracking
        self.is_repositioning = False
        self.last_reposition_time = 0.0

        # -------------------------------------------------------------
        # 2. Publishers & Subscriptions
        # -------------------------------------------------------------
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pub_capture = self.create_publisher(
            String,
            '/capture_evidence',
            10
        )

        self.sub_uncertain = self.create_subscription(
            String,
            '/evidence_candidates_uncertain',
            self.uncertain_callback,
            10
        )

        self.get_logger().info(
            f'Active Repositioning Node initialized | angle={self.angle_deg:.1f}deg | '
            f'speed={self.angular_speed:.2f}rad/s | cooldown={self.cooldown_sec:.1f}s'
        )

    def uncertain_callback(self, msg: String):
        now = time.time()
        if self.is_repositioning or (now - self.last_reposition_time) < self.cooldown_sec:
            return

        payload = msg.data.strip()
        # Parse payload: e.g. "potential_evidence:knife:0.65:..."
        parts = payload.split(':')
        obj_type = "candidate_object"
        conf = "0.50"

        if len(parts) >= 3:
            obj_type = parts[1]
            conf = parts[2]
        elif len(parts) == 2:
            obj_type = parts[0]
            conf = parts[1]

        self.get_logger().info(
            f'Uncertain evidence candidate received: [{obj_type}] (conf={conf}). Initiating controlled viewpoint change.'
        )
        self.execute_repositioning(obj_type, conf)

    def execute_repositioning(self, obj_type: str, conf: str):
        self.is_repositioning = True
        self.last_reposition_time = time.time()

        # Calculate rotation duration based on angle and angular speed
        angle_rad = math.radians(abs(self.angle_deg))
        duration_sec = angle_rad / max(self.angular_speed, 0.05)
        
        # Determine rotation direction (alternate or fixed positive rotation)
        angular_z = self.angular_speed if self.angle_deg >= 0 else -self.angular_speed

        self.get_logger().info(
            f'Executing viewpoint rotation: {self.angle_deg:.1f}deg over {duration_sec:.2f}s (w={angular_z:.2f} rad/s)'
        )

        # 1. Command controlled rotation
        twist_cmd = Twist()
        twist_cmd.angular.z = angular_z
        
        step_dt = 0.05
        steps = int(duration_sec / step_dt)
        for _ in range(steps):
            self.pub_cmd_vel.publish(twist_cmd)
            time.sleep(step_dt)

        # 2. Explicit Stop Command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        for _ in range(5):
            self.pub_cmd_vel.publish(stop_cmd)
            time.sleep(0.05)

        # 3. Allow chassis / camera to settle
        time.sleep(self.settle_time)

        # 4. Trigger Secondary Viewpoint Evidence Capture
        self.get_logger().info(
            f'Vantage point repositioning complete. Triggering secondary image capture for [{obj_type}].'
        )
        capture_msg = String()
        capture_msg.data = f"potential_evidence (repositioned viewpoint):{obj_type}:{conf}"
        self.pub_capture.publish(capture_msg)

        self.is_repositioning = False


def main(args=None):
    rclpy.init(args=args)
    node = ActiveRepositioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
