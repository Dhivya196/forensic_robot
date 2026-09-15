import csv
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node


class EvidenceCapture(Node):
    def __init__(self):
        super().__init__('evidence_capture')

        self.output_dir = Path.home() / 'forensic_evidence'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'evidence_log.csv'

        if not self.csv_path.exists():
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'evidence_id', 'timestamp', 'image_path',
                    'x', 'y', 'z', 'confidence', 'status'
                ])

        self.cap = cv2.VideoCapture(0)
        self.counter = 1

        if self.cap.isOpened():
            self.get_logger().info(
                'Camera ready. Press C to capture, Q to quit.'
            )
        else:
            self.get_logger().error('Could not open camera.')

    def capture(self, frame):
        now = datetime.now().isoformat(timespec='seconds')
        evidence_id = f'E{self.counter:03d}'
        filename = f'{evidence_id}_{now.replace(":", "-")}.jpg'
        image_path = self.output_dir / filename

        cv2.imwrite(str(image_path), frame)

        # Placeholder until SLAM/TF is connected.
        x, y, z = 0.0, 0.0, 0.0

        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                evidence_id, now, str(image_path),
                x, y, z, '', 'requires investigator verification'
            ])

        self.get_logger().info(f'Saved {evidence_id}: {image_path}')
        self.counter += 1

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                break

            cv2.imshow('Forensic Robot Camera', frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('c'):
                self.capture(frame)
            elif key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = EvidenceCapture()
    if node.cap.isOpened():
        node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
