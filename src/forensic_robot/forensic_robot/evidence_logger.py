#!/usr/bin/env python3
import csv
from datetime import datetime
import math
import os
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Graceful optional imports for CV & TF
HAS_CV2 = False
HAS_BRIDGE = False
HAS_TF2 = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    pass

try:
    from cv_bridge import CvBridge
    HAS_BRIDGE = True
except ImportError:
    pass

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    HAS_TF2 = True
except ImportError:
    pass

try:
    from visualization_msgs.msg import Marker, MarkerArray
    HAS_MARKERS = True
except ImportError:
    HAS_MARKERS = False


class SpatialEvidenceLogger(Node):
    """
    ROS 2 Node for Spatial Evidence Documentation.
    Captures live camera frames, queries real-time TF robot pose (/map or /odom),
    saves forensic documentation JPEGs and structured CSV records, and publishes RViz markers.
    """

    def __init__(self):
        super().__init__('evidence_logger')

        # -------------------------------------------------------------
        # 1. Parameter Declarations
        # -------------------------------------------------------------
        self.declare_parameter('output_dir', str(Path.home() / 'forensic_evidence'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        image_topic = str(self.get_parameter('image_topic').value)

        # Ensure output directory exists
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'evidence_log.csv'

        # Initialize CSV header if file is new
        if not self.csv_path.exists():
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'evidence_id', 'timestamp', 'image_path',
                    'x', 'y', 'z', 'yaw_deg',
                    'category', 'confidence', 'status'
                ])

        # CV Bridge & Image Buffer
        self.bridge = CvBridge() if HAS_BRIDGE else None
        self.latest_cv_image = None
        self.counter = self._determine_initial_counter()

        # TF2 Listener
        self.tf_buffer = None
        self.tf_listener = None
        if HAS_TF2:
            try:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            except Exception as e:
                self.get_logger().warn(f'TF2 listener initialization failed: {e}')

        # Marker Array for RViz visualization
        if HAS_MARKERS:
            self.marker_array = MarkerArray()
            self.pub_markers = self.create_publisher(
                MarkerArray,
                '/evidence_markers',
                10
            )
            self.timer_marker_repub = self.create_timer(1.0, self.publish_all_markers)

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.sub_trigger = self.create_subscription(
            String,
            '/capture_evidence',
            self.trigger_callback,
            10
        )

        self.get_logger().info(
            f'Spatial Evidence Logger active | Directory: {self.output_dir} | CSV: {self.csv_path}'
        )

    def _determine_initial_counter(self) -> int:
        """Determines next evidence ID by scanning existing CSV entries."""
        if not self.csv_path.exists():
            return 1
        count = 0
        try:
            with open(self.csv_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for _ in reader:
                    count += 1
        except Exception:
            count = 0
        return count + 1

    def image_callback(self, msg: Image):
        if not HAS_CV2 or self.bridge is None:
            return
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')

    def trigger_callback(self, msg: String):
        """
        Parses trigger message format:
        e.g., 'potential_evidence:knife:0.87:120,150,300,400'
        or 'knife:0.87' or 'potential_evidence (repositioned viewpoint):knife:0.75'
        """
        payload = msg.data.strip()
        category = "potential_evidence"
        confidence = "1.00"
        status = "potential"

        if ':' in payload:
            parts = payload.split(':')
            if len(parts) == 2:
                category = parts[0].strip()
                confidence = parts[1].strip()
            elif len(parts) >= 3:
                # Format: potential_evidence:knife:0.87:...
                prefix = parts[0].strip()
                obj_type = parts[1].strip()
                confidence = parts[2].strip()
                category = obj_type if prefix == "potential_evidence" else f"{prefix} ({obj_type})"
        elif payload:
            category = payload

        self.capture_evidence(category=category, confidence=confidence, status=status)

    def capture_evidence(self, category="knife", confidence="0.87", status="potential"):
        evidence_id = f'E{self.counter:03d}'
        now = datetime.now()
        timestamp_str = now.strftime('%Y-%m-%d %H:%M:%S')
        file_timestamp = now.strftime('%Y%m%d_%H%M%S')
        filename = f'{evidence_id}_{file_timestamp}.jpg'
        image_path = self.output_dir / filename

        # 1. Save Camera Frame Image to Disk
        if self.latest_cv_image is not None and HAS_CV2:
            cv2.imwrite(str(image_path), self.latest_cv_image)
        else:
            # Fallback placeholder if hardware camera not streaming yet
            image_path.touch(exist_ok=True)
            self.get_logger().warn(f'Camera frame not available; created placeholder image: {filename}')

        # 2. Lookup Robot Pose from TF (/map -> base_footprint or /odom -> base_footprint)
        x, y, z, yaw_deg, pose_status = self._get_current_pose()
        if pose_status != "verified":
            status = "potential (estimated pose)"

        # 3. Write CSV Record to Audit Log
        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                evidence_id, timestamp_str, str(image_path),
                f'{x:.2f}', f'{y:.2f}', f'{z:.1f}', f'{yaw_deg:.1f}',
                category, confidence, status
            ])

        # 4. Add RViz 3D Marker
        if HAS_MARKERS and hasattr(self, 'pub_markers'):
            self._add_rviz_marker(self.counter, evidence_id, category, confidence, x, y, z)

        self.get_logger().info(
            f'>>> EVIDENCE DOCUMENTED: [{evidence_id}] category={category} conf={confidence} '
            f'pose=(x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}deg) -> {filename}'
        )
        self.counter += 1

    def _get_current_pose(self):
        """Queries TF for robot coordinates in the global map frame or odom frame."""
        if self.tf_buffer is not None:
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
                    siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
                    cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    return t.x, t.y, t.z, math.degrees(yaw), "verified"
                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue

        # Non-blocking fallback estimate when SLAM/TF is not active
        return 0.0, 0.0, 0.0, 0.0, "estimated"

    def _add_rviz_marker(self, marker_id, evidence_id, category, confidence, x, y, z):
        if not HAS_MARKERS:
            return

        # 1. Sphere Marker
        pin = Marker()
        pin.header.frame_id = self.global_frame
        pin.header.stamp = self.get_clock().now().to_msg()
        pin.ns = "potential_evidence_pins"
        pin.id = marker_id * 2
        pin.type = Marker.SPHERE
        pin.action = Marker.ADD
        pin.pose.position.x = float(x)
        pin.pose.position.y = float(y)
        pin.pose.position.z = float(z) + 0.05
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
        text.ns = "potential_evidence_labels"
        text.id = marker_id * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = float(z) + 0.30
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{evidence_id}: {category} ({confidence})"

        self.marker_array.markers.append(pin)
        self.marker_array.markers.append(text)
        self.pub_markers.publish(self.marker_array)

    def publish_all_markers(self):
        if HAS_MARKERS and hasattr(self, 'pub_markers') and self.marker_array.markers:
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
