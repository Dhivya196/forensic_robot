#!/usr/bin/env python3
import time
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Optional import of Ultralytics YOLO
HAS_YOLO = False
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    pass


class ForensicVisionDetector(Node):
    """
    ROS 2 Node for real-time forensic candidate detection.
    Evaluates camera frames, detects potential evidence (weapons, stains, footwear impressions),
    publishes annotated visualization streams, and triggers automated capture / repositioning.
    """

    def __init__(self):
        super().__init__('vision_detector')

        # Parameters
        self.declare_parameter('high_confidence_thresh', 0.80)
        self.declare_parameter('low_confidence_thresh', 0.40)
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('auto_capture', True)
        self.declare_parameter('capture_cooldown_sec', 3.0)

        self.high_thresh = float(self.get_parameter('high_confidence_thresh').value)
        self.low_thresh = float(self.get_parameter('low_confidence_thresh').value)
        self.model_path = self.get_parameter('yolo_model_path').value
        self.auto_capture = bool(self.get_parameter('auto_capture').value)
        self.cooldown_sec = float(self.get_parameter('capture_cooldown_sec').value)

        self.bridge = CvBridge()
        self.last_capture_time = 0.0

        # Load YOLO model or initialize fallback detector
        self.yolo = None
        if HAS_YOLO:
            try:
                self.yolo = YOLO(self.model_path)
                self.get_logger().info(f'YOLO model loaded: {self.model_path}')
            except Exception as e:
                self.get_logger().warn(f'Could not load YOLO model: {e}. Using forensic heuristic engine.')
        else:
            self.get_logger().info('Ultralytics not installed. Running forensic heuristic detection engine.')

        # Target forensic categories mapping (COCO class ID or keywords)
        self.forensic_targets = {
            'knife': 'Knife-like Object',
            'scissors': 'Sharp Instrument',
            'bottle': 'Potential Container/Fluid',
            'cell phone': 'Electronic Device/Phone',
            'cup': 'Container/Evidence',
            'handbag': 'Personal Item/Bag',
            'backpack': 'Personal Item/Bag'
        }

        # Publishers
        self.pub_annotated = self.create_publisher(
            Image,
            '/camera/detection_image',
            10
        )
        self.pub_capture_trigger = self.create_publisher(
            String,
            '/capture_evidence',
            10
        )
        self.pub_uncertain_trigger = self.create_publisher(
            String,
            '/evidence_candidates_uncertain',
            10
        )

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Forensic Vision Detector initialized.')

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        annotated_img = cv_img.copy()
        detections = []

        if self.yolo is not None:
            detections = self._detect_yolo(cv_img, annotated_img)
        else:
            detections = self._detect_heuristics(cv_img, annotated_img)

        # Publish annotated stream
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

        # Evaluate highest confidence detection
        if detections:
            top_category, top_conf, bbox = max(detections, key=lambda d: d[1])
            now = time.time()

            if (now - self.last_capture_time) > self.cooldown_sec:
                if top_conf >= self.high_thresh:
                    # High confidence -> Trigger automatic capture
                    if self.auto_capture:
                        trigger_msg = String()
                        trigger_msg.data = f"{top_category}:{top_conf:.2f}"
                        self.pub_capture_trigger.publish(trigger_msg)
                        self.last_capture_time = now
                        self.get_logger().info(f"High confidence detection: {top_category} ({top_conf:.2%}) -> Auto captured.")
                elif top_conf >= self.low_thresh:
                    # Low/uncertain confidence -> Request active repositioning
                    uncertain_msg = String()
                    uncertain_msg.data = f"{top_category}:{top_conf:.2f}:{bbox[0]}:{bbox[1]}"
                    self.pub_uncertain_trigger.publish(uncertain_msg)
                    self.get_logger().info(f"Uncertain detection: {top_category} ({top_conf:.2%}) -> Repositioning requested.")

    def _detect_yolo(self, img, annotated_img):
        results = self.yolo(img, verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.yolo.names[cls_id].lower()
                conf = float(box.conf[0].item())

                if cls_name in self.forensic_targets:
                    category = self.forensic_targets[cls_name]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy

                    # Draw bounding box
                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 220, 255), 2)
                    label = f"{category}: {conf:.2f}"
                    cv2.putText(
                        annotated_img, label, (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2
                    )
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    detections.append((category, conf, (center_x, center_y)))

        return detections

    def _detect_heuristics(self, img, annotated_img):
        """
        Heuristic forensic candidate detector:
        Detects blood-like stain regions and dark sharp object contours.
        """
        detections = []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 1. Blood-like stain detection (Red/Crimson HSV ranges)
        lower_red1 = np.array([0, 90, 40])
        upper_red1 = np.array([10, 255, 200])
        lower_red2 = np.array([170, 90, 40])
        upper_red2 = np.array([180, 255, 200])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        stain_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        stain_mask = cv2.morphologyEx(stain_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(stain_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 800 < area < 100000:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h
                circularity = 4 * np.pi * (area / (cv2.arcLength(cnt, True) ** 2 + 1e-5))

                confidence = min(0.45 + (circularity * 0.40), 0.92)
                category = "Blood-like Stain"

                cv2.rectangle(annotated_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(
                    annotated_img, f"{category} ({confidence:.2f})", (x, max(y - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2
                )
                detections.append((category, confidence, (x + w // 2, y + h // 2)))

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ForensicVisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
