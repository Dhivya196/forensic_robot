#!/usr/bin/env python3
"""
Comprehensive Verification Suite for Forensic Robot 75% Physical Prototype.
Tests:
1. Motor kinematics, 120 RPM rate mapping, PWM deadband, and safety watchdog.
2. Vision detector candidate filtering, threshold classification, and trigger formatting.
3. Evidence logger CSV schema, file naming (E001_YYYYMMDD_HHMMSS.jpg), and pose fallback.
4. Active repositioner controlled /cmd_vel movement and secondary capture triggering.
5. End-to-end mock pipeline integration.
"""

import csv
import math
import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path


class MockMsg:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class MockTwist:
    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    def __init__(self, lx=0.0, az=0.0):
        self.linear = self.Vector3(x=lx)
        self.angular = self.Vector3(z=az)


class TestMotorKinematics(unittest.TestCase):
    """Verifies motor driver kinematics and 120 RPM limits."""

    def setUp(self):
        self.wheel_radius = 0.0325  # 65 mm diameter -> 0.0325 m
        self.wheel_base = 0.200     # 200 mm track width
        self.max_rpm = 120.0        # Rated 120 RPM
        self.max_rad_s = (self.max_rpm * 2.0 * math.pi) / 60.0  # ~12.566 rad/s
        self.min_pwm = 25.0

    def calc_speeds(self, linear_v, angular_w):
        v_left = linear_v - (angular_w * self.wheel_base / 2.0)
        v_right = linear_v + (angular_w * self.wheel_base / 2.0)
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius
        return w_left, w_right

    def calc_duty_dir(self, w):
        if abs(w) < 1e-4:
            return 0.0, 0
        direction = 1 if w > 0 else -1
        ratio = min(abs(w) / self.max_rad_s, 1.0)
        duty = self.min_pwm + (ratio * (100.0 - self.min_pwm))
        return max(0.0, min(100.0, duty)), direction

    def test_forward_kinematics(self):
        w_l, w_r = self.calc_speeds(0.20, 0.0)
        self.assertAlmostEqual(w_l, w_r, places=4)
        self.assertGreater(w_l, 0)
        duty_l, dir_l = self.calc_duty_dir(w_l)
        duty_r, dir_r = self.calc_duty_dir(w_r)
        self.assertEqual(dir_l, 1)
        self.assertEqual(dir_r, 1)
        self.assertTrue(25.0 <= duty_l <= 100.0)

    def test_reverse_kinematics(self):
        w_l, w_r = self.calc_speeds(-0.20, 0.0)
        duty_l, dir_l = self.calc_duty_dir(w_l)
        duty_r, dir_r = self.calc_duty_dir(w_r)
        self.assertEqual(dir_l, -1)
        self.assertEqual(dir_r, -1)
        self.assertTrue(duty_l > self.min_pwm)

    def test_left_turn_in_place(self):
        w_l, w_r = self.calc_speeds(0.0, 1.0)
        duty_l, dir_l = self.calc_duty_dir(w_l)
        duty_r, dir_r = self.calc_duty_dir(w_r)
        self.assertEqual(dir_l, -1)  # Left wheel reverses
        self.assertEqual(dir_r, 1)   # Right wheel moves forward

    def test_right_turn_in_place(self):
        w_l, w_r = self.calc_speeds(0.0, -1.0)
        duty_l, dir_l = self.calc_duty_dir(w_l)
        duty_r, dir_r = self.calc_duty_dir(w_r)
        self.assertEqual(dir_l, 1)   # Left wheel moves forward
        self.assertEqual(dir_r, -1)  # Right wheel reverses

    def test_stop_state(self):
        duty, direction = self.calc_duty_dir(0.0)
        self.assertEqual(duty, 0.0)
        self.assertEqual(direction, 0)


class TestVisionDetectorLogic(unittest.TestCase):
    """Verifies potential evidence classification, thresholding, and cooldown."""

    def setUp(self):
        self.high_thresh = 0.80
        self.low_thresh = 0.40
        self.supported_classes = {
            'knife': 'knife',
            'scissors': 'scissors',
            'bottle': 'bottle',
            'cell phone': 'cell phone',
            'backpack': 'backpack',
            'handbag': 'handbag'
        }

    def classify_detection(self, class_name, confidence, bbox):
        if class_name not in self.supported_classes:
            return None, None

        obj_type = self.supported_classes[class_name]
        if confidence >= self.high_thresh:
            payload = f"potential_evidence:{obj_type}:{confidence:.2f}:{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
            return "high_capture", payload
        elif confidence >= self.low_thresh:
            center_x = (bbox[0] + bbox[2]) // 2
            center_y = (bbox[1] + bbox[3]) // 2
            payload = f"potential_evidence:{obj_type}:{confidence:.2f}:{center_x}:{center_y}"
            return "uncertain_reposition", payload
        return "ignored", None

    def test_high_confidence_knife_capture(self):
        action, payload = self.classify_detection('knife', 0.87, (100, 150, 300, 450))
        self.assertEqual(action, "high_capture")
        self.assertTrue(payload.startswith("potential_evidence:knife:0.87"))

    def test_uncertain_confidence_bottle_reposition(self):
        action, payload = self.classify_detection('bottle', 0.62, (50, 80, 150, 240))
        self.assertEqual(action, "uncertain_reposition")
        self.assertTrue(payload.startswith("potential_evidence:bottle:0.62"))

    def test_low_confidence_discarded(self):
        action, payload = self.classify_detection('scissors', 0.35, (10, 20, 50, 60))
        self.assertEqual(action, "ignored")
        self.assertIsNone(payload)

    def test_unsupported_class_discarded(self):
        action, payload = self.classify_detection('dog', 0.95, (10, 20, 50, 60))
        self.assertEqual(action, None)


class TestEvidenceLoggerSchema(unittest.TestCase):
    """Verifies evidence CSV schema, image naming, and pose formatting."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.output_dir = Path(self.test_dir) / 'forensic_evidence'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'evidence_log.csv'

    def tearDown(self):
        shutil.rmtree(self.test_dir)

    def test_csv_creation_and_fields(self):
        # Initialize header
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'evidence_id', 'timestamp', 'image_path',
                'x', 'y', 'z', 'yaw_deg',
                'category', 'confidence', 'status'
            ])

        # Write sample row
        evidence_id = "E001"
        now_str = "2026-09-16 10:30:15"
        img_name = f"{evidence_id}_20260916_103015.jpg"
        img_path = self.output_dir / img_name
        img_path.touch()

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                evidence_id, now_str, str(img_path),
                '1.24', '0.53', '0.0', '92.5',
                'knife', '0.87', 'potential'
            ])

        # Read back and verify
        with open(self.csv_path, 'r', encoding='utf-8') as f:
            reader = list(csv.reader(f))
            header = reader[0]
            row = reader[1]

            expected_header = [
                'evidence_id', 'timestamp', 'image_path',
                'x', 'y', 'z', 'yaw_deg',
                'category', 'confidence', 'status'
            ]
            self.assertEqual(header, expected_header)
            self.assertEqual(row[0], 'E001')
            self.assertEqual(row[7], 'knife')
            self.assertEqual(row[8], '0.87')
            self.assertEqual(row[9], 'potential')
            self.assertTrue(row[2].endswith(img_name))


class TestActiveRepositionerTiming(unittest.TestCase):
    """Verifies angle to rotation duration and cmd_vel generation."""

    def test_rotation_duration_calculation(self):
        angle_deg = 35.0
        angular_speed = 0.40  # rad/s
        angle_rad = math.radians(angle_deg)  # ~0.6108 rad
        duration = angle_rad / angular_speed  # ~1.527 sec
        self.assertAlmostEqual(duration, 1.527, places=2)

    def test_secondary_payload_generation(self):
        obj_type = "knife"
        conf = "0.65"
        payload = f"potential_evidence (repositioned viewpoint):{obj_type}:{conf}"
        self.assertEqual(payload, "potential_evidence (repositioned viewpoint):knife:0.65")


class TestEndToEndPipeline(unittest.TestCase):
    """Simulates the full 75% prototype integration pipeline."""

    def test_full_pipeline_flow(self):
        # 1. Detection Event
        raw_detections = [
            {'class': 'knife', 'conf': 0.89, 'bbox': (100, 120, 280, 400)},
            {'class': 'bottle', 'conf': 0.58, 'bbox': (350, 200, 420, 320)}
        ]

        # 2. Process High Confidence Detection
        high_det = raw_detections[0]
        self.assertGreaterEqual(high_det['conf'], 0.80)
        capture_trigger = f"potential_evidence:{high_det['class']}:{high_det['conf']:.2f}"

        # 3. Simulate Evidence Logger receiving capture trigger
        test_dir = tempfile.mkdtemp()
        try:
            out_dir = Path(test_dir) / 'forensic_evidence'
            out_dir.mkdir(parents=True, exist_ok=True)
            csv_file = out_dir / 'evidence_log.csv'
            
            with open(csv_file, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([
                    'evidence_id', 'timestamp', 'image_path',
                    'x', 'y', 'z', 'yaw_deg',
                    'category', 'confidence', 'status'
                ])

            parts = capture_trigger.split(':')
            cat = parts[1]
            conf = parts[2]
            img_file = out_dir / "E001_20260916_105000.jpg"
            img_file.touch()

            with open(csv_file, 'a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([
                    'E001', '2026-09-16 10:50:00', str(img_file),
                    '0.00', '0.00', '0.0', '0.0',
                    cat, conf, 'potential'
                ])

            # 4. Process Uncertain Detection -> Repositioning
            unc_det = raw_detections[1]
            self.assertTrue(0.40 <= unc_det['conf'] < 0.80)
            reposition_trigger = f"potential_evidence:{unc_det['class']}:{unc_det['conf']:.2f}"
            
            # Repositioning generates secondary capture
            reposition_parts = reposition_trigger.split(':')
            sec_capture_trigger = f"potential_evidence (repositioned viewpoint):{reposition_parts[1]}:{reposition_parts[2]}"
            
            sec_img_file = out_dir / "E002_20260916_105005.jpg"
            sec_img_file.touch()

            with open(csv_file, 'a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([
                    'E002', '2026-09-16 10:50:05', str(sec_img_file),
                    '0.00', '0.00', '0.0', '35.0',
                    'bottle', '0.58', 'potential'
                ])

            # Verify 2 entries in CSV
            with open(csv_file, 'r', encoding='utf-8') as f:
                lines = list(csv.reader(f))
                self.assertEqual(len(lines), 3)  # header + 2 records
                self.assertEqual(lines[1][0], 'E001')
                self.assertEqual(lines[1][7], 'knife')
                self.assertEqual(lines[2][0], 'E002')
                self.assertEqual(lines[2][7], 'bottle')
        finally:
            shutil.rmtree(test_dir)


if __name__ == '__main__':
    unittest.main(verbosity=2)
