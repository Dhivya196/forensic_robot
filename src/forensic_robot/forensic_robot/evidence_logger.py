#!/usr/bin/env python3
import csv
from datetime import datetime
import math
import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray


class SpatialEvidenceLogger(Node):
    """
    ROS 2 Node for spatial evidence logging.
    Captures camera frames, looks up real-time TF robot/sensor pose in the /map frame,
    writes forensic audit logs (CSV + JPEGs), and publishes persistent 3D RViz markers.
    """

    def __init__(self):
        super().__init__('evidence_logger')

        # Parameters
        self.declare_parameter('output_dir', str(Path.home() / 'forensic_evidence'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.images_dir = self.output_dir / 'images'
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        image_topic = self.get_parameter('image_topic').value

        # Create directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'evidence_log.csv'

        # Initialize CSV header if not exists
        if not self.csv_path.exists():
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'evidence_id', 'timestamp', 'image_path',
                    'x', 'y', 'z', 'yaw_deg',
                    'category', 'confidence', 'status'
                ])

        # CV Bridge & Image Buffer
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.counter = 1

        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker Array for RViz visualization
        self.marker_array = MarkerArray()
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/evidence_markers',
            10
        )

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Trigger subscription for manual/automated capture
        self.sub_trigger = self.create_subscription(
            String,
            '/capture_evidence',
            self.trigger_callback,
            10
        )

        # Periodic timer to republish RViz markers
        self.timer_marker_repub = self.create_timer(1.0, self.publish_all_markers)

        self.get_logger().info(
            f'Spatial Evidence Logger active. Evidence directory: {self.output_dir}'
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')

    def trigger_callback(self, msg: String):
        """
        Triggers an evidence capture.
        Message payload can optionally specify category and confidence (e.g., "knife:0.92" or "manual").
        """
        payload = msg.data.strip()
        category = "Unclassified Item"
        confidence = "1.00"

        if ':' in payload:
            parts = payload.split(':')
            category = parts[0].strip()
            confidence = parts[1].strip()
        elif payload:
            category = payload

        self.capture_evidence(category=category, confidence=confidence)

    def capture_evidence(self, category="Potential Evidence", confidence="1.00"):
        if self.latest_cv_image is None:
            self.get_logger().warn('No camera frame received yet. Cannot capture evidence.')
            return

        evidence_id = f'E{self.counter:03d}'
        now = datetime.now()
        timestamp_str = now.isoformat(timespec='seconds')
        file_timestamp = now.strftime('%Y%m%d_%H%M%S')
        filename = f'{evidence_id}_{file_timestamp}.jpg'
        image_path = self.images_dir / filename

        # 1. Save Image to Disk
        cv2.imwrite(str(image_path), self.latest_cv_image)

        # 2. Lookup Robot Pose from TF (/map -> base_footprint or /odom -> base_footprint)
        x, y, z, yaw_deg = self._get_current_pose()

        # 3. Write to CSV Log
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                evidence_id, timestamp_str, str(image_path),
                f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{yaw_deg:.1f}',
                category, confidence, 'Requires Investigator Verification'
            ])

        # 4. Add 3D RViz Marker
        self._add_rviz_marker(self.counter, evidence_id, category, confidence, x, y, z)

        self.get_logger().info(
            f'>>> CAPTURED {evidence_id} [{category}] at (x={x:.2f}, y={y:.2f}) -> {filename}'
        )
        self.counter += 1

    def _get_current_pose(self):
        """Queries TF for robot coordinates in the global frame."""
        for target_frame in [self.global_frame, 'odom']:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2)
                )
                t = trans.transform.translation
                r = trans.transform.rotation
                # Yaw from quaternion
                siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
                cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return t.x, t.y, t.z, math.degrees(yaw)
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

        self.get_logger().warn('TF lookup failed for pose. Logging fallback coordinates (0, 0, 0).')
        return 0.0, 0.0, 0.0, 0.0

    def _add_rviz_marker(self, marker_id, evidence_id, category, confidence, x, y, z):
        # 1. Sphere Pin Marker
        pin = Marker()
        pin.header.frame_id = self.global_frame
        pin.header.stamp = self.get_clock().now().to_msg()
        pin.ns = "evidence_pins"
        pin.id = marker_id * 2
        pin.type = Marker.SPHERE
        pin.action = Marker.ADD
        pin.pose.position.x = x
        pin.pose.position.y = y
        pin.pose.position.z = z + 0.05
        pin.pose.orientation.w = 1.0
        pin.scale.x = 0.15
        pin.scale.y = 0.15
        pin.scale.z = 0.15
        pin.color.r = 1.0
        pin.color.g = 0.84
        pin.color.b = 0.0
        pin.color.a = 0.95

        # 2. Text Label Marker
        text = Marker()
        text.header.frame_id = self.global_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "evidence_labels"
        text.id = marker_id * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + 0.30
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12  # Text height
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{evidence_id}: {category} ({confidence})"

        self.marker_array.markers.append(pin)
        self.marker_array.markers.append(text)
        self.pub_markers.publish(self.marker_array)

    def publish_all_markers(self):
        if self.marker_array.markers:
            self.pub_markers.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SpatialEvidenceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
