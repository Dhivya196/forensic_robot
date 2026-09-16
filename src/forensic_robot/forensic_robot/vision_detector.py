#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Graceful optional imports for OpenCV, CvBridge, and Ultralytics YOLO
HAS_CV2 = False
HAS_BRIDGE = False
HAS_YOLO = False

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
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    pass


class ForensicVisionDetector(Node):
    """
    ROS 2 Vision Node for Potential Evidence Candidate Localization.
    
    IMPORTANT FORENSIC AI SCOPE & TERMINOLOGY:
    - This node operates an object detector using a pretrained lightweight model (e.g. YOLO26n / YOLO11n / YOLOv8n).
    - Detected supported objects are classified by application logic as 'potential evidence candidates'.
    - The system documents candidate objects with bounding boxes, confidence scores, and timestamps.
    - No claims of 'confirmed evidence', 'blood', 'fingerprints', or 'DNA' are made.
    """

    def __init__(self):
        super().__init__('vision_detector')

        # -------------------------------------------------------------
        # 1. Parameter Declarations
        # -------------------------------------------------------------
        self.declare_parameter('high_confidence_thresh', 0.80)
        self.declare_parameter('low_confidence_thresh', 0.40)
        self.declare_parameter('yolo_model_path', 'yolo11n.pt')  # e.g., yolo26n / yolo11n / yolov8n
        self.declare_parameter('auto_capture', True)
        self.declare_parameter('capture_cooldown_sec', 3.0)
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.high_thresh = float(self.get_parameter('high_confidence_thresh').value)
        self.low_thresh = float(self.get_parameter('low_confidence_thresh').value)
        self.model_path = str(self.get_parameter('yolo_model_path').value)
        self.auto_capture = bool(self.get_parameter('auto_capture').value)
        self.cooldown_sec = float(self.get_parameter('capture_cooldown_sec').value)
        image_topic = str(self.get_parameter('image_topic').value)

        self.bridge = CvBridge() if HAS_BRIDGE else None
        self.last_capture_time = 0.0

        # Supported COCO classes mapped into potential evidence candidate objects
        self.supported_evidence_classes = {
            'knife': 'knife',
            'scissors': 'scissors',
            'bottle': 'bottle',
            'cell phone': 'cell phone',
            'backpack': 'backpack',
            'handbag': 'handbag',
            'cup': 'cup',
            'laptop': 'laptop',
            'suitcase': 'suitcase'
        }

        # -------------------------------------------------------------
        # 2. YOLO Model Initialization
        # -------------------------------------------------------------
        self.yolo = None
        if HAS_YOLO:
            try:
                self.yolo = YOLO(self.model_path)
                self.get_logger().info(
                    f'Loaded YOLO model: {self.model_path} for potential evidence candidate detection.'
                )
            except Exception as e:
                self.get_logger().warn(f'Could not load YOLO model ({self.model_path}): {e}. Using fallback.')
        else:
            self.get_logger().warn('Ultralytics YOLO not installed. Vision detector in standby mode.')

        # -------------------------------------------------------------
        # 3. Publishers & Subscriptions
        # -------------------------------------------------------------
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

        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(
            f'Forensic Vision Detector initialized | high_thresh={self.high_thresh:.2f} | '
            f'low_thresh={self.low_thresh:.2f} | cooldown={self.cooldown_sec:.1f}s'
        )

    def image_callback(self, msg: Image):
        if not HAS_CV2 or self.bridge is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge image conversion error: {e}')
            return

        annotated_img = cv_img.copy()
        detections = []

        if self.yolo is not None:
            detections = self._detect_yolo(cv_img, annotated_img)

        # Publish annotated detection stream
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

        # Evaluate highest confidence detection
        if detections:
            obj_type, conf, bbox = max(detections, key=lambda d: d[1])
            now = time.time()

            if (now - self.last_capture_time) > self.cooldown_sec:
                if conf >= self.high_thresh:
                    # High confidence potential evidence candidate -> Automatic Capture Trigger
                    if self.auto_capture:
                        trigger_msg = String()
                        trigger_msg.data = f"potential_evidence:{obj_type}:{conf:.2f}:{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
                        self.pub_capture_trigger.publish(trigger_msg)
                        self.last_capture_time = now
                        self.get_logger().info(
                            f'Potential evidence candidate detected: {obj_type} (conf={conf:.2f} >= {self.high_thresh:.2f}) -> Triggering capture.'
                        )
                elif conf >= self.low_thresh:
                    # Uncertain potential evidence candidate -> Request Active Repositioning
                    uncertain_msg = String()
                    uncertain_msg.data = f"potential_evidence:{obj_type}:{conf:.2f}:{bbox[0]}:{bbox[1]}"
                    self.pub_uncertain_trigger.publish(uncertain_msg)
                    self.last_capture_time = now
                    self.get_logger().info(
                        f'Uncertain evidence candidate detected: {obj_type} (conf={conf:.2f}) -> Requesting active repositioning.'
                    )

    def _detect_yolo(self, img, annotated_img):
        """Runs YOLO inference and annotates potential evidence candidates."""
        detections = []
        try:
            results = self.yolo(img, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return detections

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.yolo.names[cls_id].lower()
                conf = float(box.conf[0].item())

                if cls_name in self.supported_evidence_classes:
                    obj_type = self.supported_evidence_classes[cls_name]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

                    # Draw Bounding Box and Candidate Label
                    if HAS_CV2:
                        color = (0, 220, 255) if conf >= self.high_thresh else (0, 165, 255)
                        cv2.rectangle(annotated_img, (x1, y1), (x2, y2), color, 2)
                        label = f"Potential {obj_type.capitalize()}: {conf:.2f}"
                        cv2.putText(
                            annotated_img, label, (x1, max(y1 - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
                        )
                    detections.append((obj_type, conf, (x1, y1, x2, y2)))

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
