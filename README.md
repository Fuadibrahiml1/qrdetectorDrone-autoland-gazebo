```
 Autonomous QR Landing — Gazebo SITL

A minimal setup & run guide for qr_autoland.py (Iris in Gazebo SITL).

Requirements:
- Ubuntu (or similar), ArduPilot SITL + Gazebo, Python 3.8+

Install Python dependencies:
pip3 install dronekit pymavlink opencv-python numpy

Start SITL + Gazebo (example):
~/ardupilot/Tools/autotest$ sim_vehicle.py -v ArduCopter -f gazebo-iris --model=+ --console --map

Run the script (in another terminal):
python3 qr_autoland.py

Notes:
- Default connection: udp:127.0.0.1:14550 .
- Default camera: CAMERA_SOURCE=0 — for Gazebo camera streamchange 0 to URL.
- The script shows an OpenCV window; press ESC to abort. Test in SITL first.
```
