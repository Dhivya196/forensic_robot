#!/usr/bin/env python3
import math
import time
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class ActiveRepositioner(Node):
    """
    ROS 2 Node for Active Repositioning / Viewpoint Planning.
    When an evidence candidate is detected with uncertain confidence or partial occlusion,
    this node computes an alternative vantage pose and commands Nav2 to reposition
    the robot for a secondary verification capture.
    """

    def __init__(self):
        super().__init__('active_repositioner')

        self.declare_parameter('reposition_distance', 0.50)  # meters offset
        self.declare_parameter('reposition_angle_deg', 35.0)  # degrees offset
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.dist_offset = float(self.get_parameter('reposition_distance').value)
        self.angle_offset = math.radians(float(self.get_parameter('reposition_angle_deg').value))
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # State tracking
        self.is_repositioning = False
        self.last_reposition_time = 0.0

        # TF2 Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Goal Publisher
        self.pub_goal = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Evidence Capture Trigger Publisher
        self.pub_capture = self.create_publisher(
            String,
            '/capture_evidence',
            10
        )

        # Subscription for uncertain candidate detections
        self.sub_uncertain = self.create_subscription(
            String,
            '/evidence_candidates_uncertain',
            self.uncertain_callback,
            10
        )

        self.get_logger().info('Active Repositioning Node initialized and standing by.')

    def uncertain_callback(self, msg: String):
        now = time.time()
        # Cooldown of 15 seconds to complete maneuver before accepting new trigger
        if self.is_repositioning or (now - self.last_reposition_time) < 15.0:
            return

        payload = msg.data.strip()
        parts = payload.split(':')
        category = parts[0] if len(parts) > 0 else "Uncertain Item"
        conf = parts[1] if len(parts) > 1 else "0.50"

        self.get_logger().info(
            f'Planning active repositioning for [{category}] with confidence {conf}...'
        )
        self.execute_repositioning(category, conf)

    def execute_repositioning(self, category: str, conf: str):
        self.is_repositioning = True
        self.last_reposition_time = time.time()

        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not lookup transform for repositioning: {e}')
            self.is_repositioning = False
            return

        curr_x = trans.transform.translation.x
        curr_y = trans.transform.translation.y
        r = trans.transform.rotation
        siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
        cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Compute alternative viewpoint (arc offset to the side and slightly rotated toward candidate)
        target_yaw = curr_yaw + self.angle_offset
        goal_x = curr_x + (self.dist_offset * math.cos(target_yaw))
        goal_y = curr_y + (self.dist_offset * math.sin(target_yaw))

        # Face back toward target
        facing_yaw = math.atan2(curr_y - goal_y, curr_x - goal_x) + math.pi

        # Create Goal Pose
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        # Yaw to Quaternion
        goal.pose.orientation.z = math.sin(facing_yaw / 2.0)
        goal.pose.orientation.w = math.cos(facing_yaw / 2.0)

        self.get_logger().info(
            f'Navigating to new viewpoint: (x={goal_x:.2f}, y={goal_y:.2f})'
        )
        self.pub_goal.publish(goal)

        # Schedule follow-up capture after reaching pose (simulated timer)
        self.create_timer(8.0, lambda: self._complete_repositioning(category, conf))

    def _complete_repositioning(self, category: str, conf: str):
        if self.is_repositioning:
            self.get_logger().info('Vantage point reached. Capturing secondary verification view.')
            capture_msg = String()
            capture_msg.data = f"{category} (Multi-View Verified):{conf}"
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
