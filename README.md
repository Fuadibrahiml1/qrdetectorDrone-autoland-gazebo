# QR-Detector Autonomous Autoland for Iris (Gazebo SITL, ArduPilot, DroneKit & OpenCV)

Autonomous QR-code landing demo for the Iris quadrotor in Gazebo SITL using ArduPilot, DroneKit (Python) and OpenCV. The project demonstrates a complete simulation pipeline that locates a visual QR marker with a downward-facing camera, guides the vehicle over the marker, and performs an automated landing.

Key goals:
- Reliable QR detection and pose estimation in simulation
- Closed‑loop guidance to land the vehicle accurately on the marker
- Reproducible simulation instructions for testing and development

---

## Table of contents
- [Features](#features)
- [Repository layout](#repository-layout)
- [Requirements](#requirements)
- [Installation and setup](#installation-and-setup)
- [Running the demo (quick start)](#running-the-demo-quick-start)
- [How it works (high level)](#how-it-works-high-level)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Development & testing](#development--testing)
- [License & attribution](#license--attribution)

---

## Features
- Real-time QR code detection using OpenCV / pyzbar
- DroneKit-based vehicle control connected to ArduPilot SITL
- Integration with Gazebo camera sensor for realistic visual input
- Configurable descent and landing acceptance criteria
- Safety abort conditions and basic checks for simulation use

---

## Repository layout
- qr_autoland.py — main autonomous detection + landing script (Python)
- worlds/iris_runway.sdf — Gazebo world containing runway and QR target
- README.md — this document
- (other helper scripts & assets) — camera models, marker images, utilities

---

## Requirements

Minimum tested environment
- Ubuntu 20.04 / 22.04 (recommended for ROS/Gazebo compatibility)
- Python 3.8+
- ArduPilot (ArduCopter) source and SITL tools (sim_vehicle.py)
- Gazebo (compatible with your ArduPilot build; Gazebo 11 commonly used)
- MAVProxy (optional; used by sim_vehicle.py when launched with --console/--map)

Python packages (example):
- dronekit
- pymavlink
- opencv-python or opencv-contrib-python
- numpy
- pyzbar (optional; alternative: OpenCV QRDetector)
- imutils (optional)

Example environment setup
```bash
python3 -m venv venv
source venv/bin/activate
pip install -U pip
pip install dronekit pymavlink opencv-python numpy pyzbar imutils
```

System packages (may be required on Ubuntu)
```bash
sudo apt update
sudo apt install -y ffmpeg libsm6 libxrender1
```

---

## Installation and setup

1. Clone this repository
```bash
git clone https://github.com/Fuadibrahiml1/qrdetectorDrone-autoland-gazebo.git
cd qrdetectorDrone-autoland-gazebo
```

2. Prepare ArduPilot & Gazebo (examples)
- Clone and build ArduPilot following official docs: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- Ensure the Gazebo plugin for ArduPilot is configured and that your ArduPilot SITL can connect to Gazebo.

3. Activate Python virtual environment and install packages (see Requirements).

---

## Running the demo (quick start)

These example commands reproduce the environment used for development. Run them in separate terminals as indicated.

1. Start Gazebo with the runway world (from the `worlds/` directory)
```bash
# Terminal 1: open in the repo's worlds directory (or give full path)
cd worlds
gz sim -v4 -r iris_runway.sdf
```

2. Start ArduPilot SITL and connect it to Gazebo
```bash
# Terminal 2: example path to ArduCopter in your ArduPilot checkout
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
This launches SITL (MAVProxy console), map, and the connection that bridges to Gazebo.

3. Run the QR autoland script
```bash
# Terminal 3: from the repository root
python3 qr_autoland.py
```

Note: If your script expects a specific connection string (e.g. udp:127.0.0.1:14550) or CLI flags, supply them as needed:
```bash
python3 qr_autoland.py --connect udp:127.0.0.1:14550 --approach-alt 10 --land-alt 0.5
```
Check the top of `qr_autoland.py` for the implemented CLI or add ArgumentParser flags for flexibility.

---

## How it works (high level)

1. The script connects to the ArduPilot SITL vehicle using DroneKit (MAVLink).
2. Camera frames are obtained from Gazebo's downward-facing camera (or an attached webcam).
3. Each frame is processed to detect QR codes; detection libraries (pyzbar or OpenCV QRDetector) locate the marker and return its image coordinates and decoded content.
4. Using the vehicle altitude and simple pinhole-camera approximations (or camera calibration if available), the script estimates the QR marker's relative ground offset.
5. The script issues position or velocity setpoints (GUIDED mode) to move the vehicle over the marker; once within acceptance thresholds, the script initiates landing.

Important safety note: This code is intended for simulation only. Do not use on real hardware without extensive testing and safety interlocks.

---

## Configuration

Common configurable parameters (implement or check `qr_autoland.py`):
- --connect : MAVLink connection string (default e.g. udp:127.0.0.1:14550)
- --approach-alt : altitude (m) to begin lateral alignment (default: 10)
- --descend-rate : vertical descent rate (m/s)
- --land-alt : altitude (m) to hand off to vehicle landing (default: ~0.5)
- --accept-radius : landing acceptance radius (m)
- --detection-timeout : seconds to wait before aborting detection
- --camera-calib : optional file with camera intrinsics for pose estimation

Add these flags to the script (argparse) if not present, to make the utility more robust and user-friendly.

---

## Troubleshooting

- No connection to vehicle:
  - Confirm `sim_vehicle.py` is running and the UDP/TCP port matches the script's connection string.
  - Look at MAVProxy console for incoming connections.

- No camera frames or black images:
  - Confirm the Gazebo world contains the camera sensor and that it is publishing.
  - Verify the script points to the correct video source (e.g., /dev/videoX or a Gazebo image topic).
  - Test OpenCV can open your camera separately.

- QR detection fails:
  - Increase QR marker size in the world, improve lighting, reduce camera motion blur, or tune detection thresholds.
  - Test detection on saved images to verify the algorithm.

- Vehicle drifts during approach:
  - Tune PID / controller gains in SITL or adjust approach/velocity commands in the script.
  - Use shorter control loops (reduce per-frame latency) and ensure accurate altitude estimates.

---

## Development & testing

Suggestions to improve reliability and maintainability:
- Add unit tests for image processing functions (e.g., detection and pixel-to-angle math).
- Add an integration test that runs a headless Gazebo + SITL with canned images.
- Add logging and optional image dumps for offline debugging.
- Add a requirements.txt and a Dockerfile for reproducible environments.


## License & attribution

- Include a LICENSE file in this repository. Common choices: MIT, Apache-2.0.
- Acknowledge third-party projects used: ArduPilot, DroneKit, OpenCV, pyzbar, Gazebo.

---

## Contact

Author: Fuadibrahiml1  
Repository: https://github.com/Fuadibrahiml1/qrdetectorDrone-autoland-gazebo
