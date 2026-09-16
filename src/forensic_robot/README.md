# Forensic Scene Mapping & Potential Evidence Localization Robot

An autonomous ROS 2 robotics system designed to inspect, document, and localize potential evidence candidates on indoor forensic scenes using a Raspberry Pi 4, L298N dual H-bridge motor driver, 4-wheel differential-drive kinematics with 120-RPM geared DC motors, live camera streaming, and lightweight pretrained YOLO object detection.

---

## 1. Physical Hardware Specifications & Wiring

| Component | Specification | ROS 2 Interface / Pins |
|---|---|---|
| **Main Computer** | Raspberry Pi 4 (ROS 2 Jazzy / Ubuntu 24.04 LTS) | Master ROS 2 Computation Graph |
| **Motor Driver** | L298N Dual H-Bridge Driver | Left: IN1 (GPIO24), IN2 (GPIO23), ENA (GPIO12)<br>Right: IN3 (GPIO22), IN4 (GPIO27), ENB (GPIO13) |
| **Motors** | 4 x Geared DC Motors (**120 RPM rated speed**) | Differential kinematics with deadband compensation |
| **Wheels** | $65\text{ mm}$ diameter ($r = 0.0325\text{ m}$) | Track width / Wheelbase $L = 0.200\text{ m}$ (configurable) |
| **Camera** | Pi Camera Module (CSI/V4L2) or USB Webcam | `/camera/image_raw` (`sensor_msgs/msg/Image`) |
| **Power Supply** | Separate motor battery pack + RPi 5V supply | **Common ground (GND) shared between RPi and L298N** |

> [!CAUTION]
> **Power & Grounding Rule**:
> - Raspberry Pi GPIO is used **only** for logic and PWM control signals.
> - Motor power must come from an external battery pack through the L298N power terminal.
> - Ensure a **common ground wire** connects the Raspberry Pi GND pin and the L298N GND terminal.

---

## 2. Terminology & Forensic AI Scope

The vision detector uses a general-purpose pretrained model (e.g. YOLO11n / YOLOv8n / YOLO26n) to identify standard objects and maps them through application logic into forensic candidates.

### Approved Terminology:
- `"potential evidence"`
- `"potential evidence candidate"`
- `"object detection"`
- `"candidate localization"`
- `"evidence documentation"`

### Strict Operational Guardrails:
- Do **NOT** claim `"confirmed evidence"`, `"forensic confirmation"`, `"blood detection"`, `"fingerprint detection"`, or `"DNA detection"`.
- The pretrained model is solely an object detector; forensic categorization and audit documentation are performed by the application software logic.

---

## 3. 6-Hour Physical Prototype Implementation Sequence

```
  HOUR 1: Motor Control (L298N + 120 RPM Motors + /cmd_vel + Hardware Test)
     ↓
  HOUR 2: Live Camera Stream (/camera/image_raw Verification)
     ↓
  HOUR 3: Pretrained YOLO Object Detection (Knife, Bottle, Phone Candidates)
     ↓
  HOUR 4: Spatial Evidence Logging (E001_YYYYMMDD_HHMMSS.jpg + CSV Record)
     ↓
  HOUR 5: End-to-End Motion & Evidence Integration
     ↓
  HOUR 6: Active Repositioning Demo (Uncertain Candidate -> Viewpoint Change)
```

---

### Hour 1: ROS 2 + Motor Control Verification
**Goal**: Verify `/cmd_vel` $\rightarrow$ `l298n_driver` $\rightarrow$ GPIO $\rightarrow$ L298N $\rightarrow$ 4 DC Motors.

1. Launch the physical motor driver:
   ```bash
   ros2 run forensic_robot l298n_driver
   ```
2. Run the sequential physical hardware test:
   ```bash
   ros2 run forensic_robot test_motors
   ```
   **Verification Criteria**:
   - Robot moves **FORWARD** for 2 seconds, then **STOPS**.
   - Robot moves in **REVERSE** for 2 seconds, then **STOPS**.
   - Robot rotates **LEFT** (CCW) for 2 seconds, then **STOPS**.
   - Robot rotates **RIGHT** (CW) for 2 seconds, then **STOPS**.
   - Motors stop automatically if `/cmd_vel` is interrupted (0.5s watchdog timeout).

---

### Hour 2: Camera Pipeline Bringup
**Goal**: Stream live camera frames onto `/camera/image_raw`.

1. Launch the camera driver:
   ```bash
   ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw
   ```
2. Verify image streaming rate:
   ```bash
   ros2 topic hz /camera/image_raw
   ```
   *Expected output: $\approx 15\text{--}30\text{ Hz}$ live image feed.*

---

### Hour 3: YOLO Object Detection
**Goal**: Run pretrained lightweight YOLO model and detect supported candidates (`knife`, `bottle`, `cell phone`, `scissors`, `backpack`, `handbag`).

1. Launch the vision detector node:
   ```bash
   ros2 run forensic_robot vision_detector --ros-args -p high_confidence_thresh:=0.80 -p low_confidence_thresh:=0.40
   ```
2. Observe detections:
   - High confidence ($\ge 0.80$): triggers `/capture_evidence`.
   - Uncertain confidence ($0.40 \le \text{conf} < 0.80$): triggers `/evidence_candidates_uncertain`.
   - View annotated image on `/camera/detection_image`.

---

### Hour 4: Spatial Evidence Logging
**Goal**: Save timestamped high-resolution JPEGs and append structured audit records to the CSV log.

1. Launch the spatial evidence logger:
   ```bash
   ros2 run forensic_robot evidence_logger
   ```
2. Trigger test evidence capture:
   ```bash
   ros2 topic pub --once /capture_evidence std_msgs/msg/String "{data: 'potential_evidence:knife:0.87:100,120,300,400'}"
   ```
3. Verify files generated in `~/forensic_evidence/`:
   ```bash
   ls -la ~/forensic_evidence/
   cat ~/forensic_evidence/evidence_log.csv
   ```
   **CSV Schema**:
   ```csv
   evidence_id,timestamp,image_path,x,y,z,yaw_deg,category,confidence,status
   E001,2026-09-16 10:30:15,/home/pi/forensic_evidence/E001_20260916_103015.jpg,1.24,0.53,0.0,92.5,knife,0.87,potential
   ```

---

### Hour 5: End-to-End System Bringup
**Goal**: Run motion control and perception pipelines simultaneously.

```bash
# Terminal 1: Physical Robot Bringup (Motors + Camera + TF Tree)
ros2 launch forensic_robot physical_robot.launch.py enable_lidar:=false

# Terminal 2: Evidence System (Perception + Logger + Repositioner)
ros2 launch forensic_robot evidence_system.launch.py use_sim_time:=false
```

---

### Hour 6: Active Repositioning Demonstration
**Goal**: Demonstrate automated viewpoint adjustment for uncertain candidates.

1. When an uncertain detection ($0.40 \le \text{conf} < 0.80$) is published on `/evidence_candidates_uncertain`:
   - `active_repositioner` executes a controlled rotation of $\approx 35^\circ$ via `/cmd_vel`.
   - Issues an explicit stop command.
   - Automatically triggers a secondary image capture on `/capture_evidence` with the label `potential_evidence (repositioned viewpoint)`.
2. Inspect `~/forensic_evidence/evidence_log.csv` to verify dual-viewpoint documentation.

---

## 4. Prototype Scope Comparison

| Capability | 75% Physical Prototype (Completed) | Remaining 25% (Future Work) |
|---|---|---|
| **Motor Drive** | 4-wheel differential drive with 120 RPM L298N driver | Closed-loop optical wheel encoder feedback |
| **Perception** | Pretrained lightweight YOLO candidate classification | Custom fine-tuned forensic dataset model |
| **Evidence Logging** | High-res JPEG (`E001_YYYYMMDD_HHMMSS.jpg`) + CSV audit log | Cryptographic hashing + Forensic PDF report |
| **Repositioning** | Direct controlled `/cmd_vel` rotation ($35^\circ$) | Full Nav2 costmap-aware vantage navigation |
| **Localization** | TF pose lookup with non-blocking fallback | Multi-session 2D/3D LiDAR SLAM mapping |

---

## 5. Automated Verification Tests

Run the full automated test suite to verify kinematics, thresholding, CSV schema, and pipeline integration:
```bash
python3 src/forensic_robot/forensic_robot/test_forensic_robot.py
```
