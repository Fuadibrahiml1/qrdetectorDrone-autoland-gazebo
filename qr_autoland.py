#!/usr/bin/env python3

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import time
import math
import numpy as np
import threading
import traceback

# ==================== CONFIGURATION ====================
CONNECTION_STRING = "udp:127.0.0.1:14550"
TAKEOFF_ALTITUDE = 15  
SEARCH_ALTITUDE = 12  
SEARCH_AREA_SIZE = 30 
SCAN_SPACING = 4  
SCAN_SPEED = 2  # m/s 
APPROACH_SPEED = 1  # m/s
CAMERA_SOURCE = 0  # 0 for webcam, or Gazebo camera URL

# thresholds 
QR_CENTER_THRESHOLD = 80          
PRECISION_CENTER_THRESHOLD = 30  
MIN_QR_SIZE = 800                 
LANDING_ALTITUDE = 2.5            

# Camera 
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DETECTION_INTERVAL = 0.08  


BASE_VELOCITY_SCALE = 0.006  
MAX_HORIZONTAL_VELOCITY = 1.0 



DESCENT_VELOCITY = 0.4  # m/s 
MAX_LOST_QR_FRAMES = 20 


SMOOTHING_ALPHA = 0.4  


latest_frame = None
frame_lock = threading.Lock()
stop_event = threading.Event()


smoothed_offset_x = 0.0
smoothed_offset_y = 0.0



def connect_vehicle():
    print("Connecting to vehicle:", CONNECTION_STRING)
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
    print("Connected. Mode:", vehicle.mode.name, "Armed:", vehicle.armed)
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    print("Arming and taking off to", target_altitude, "m")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Alt: {alt:.1f}/{target_altitude:.1f} m", end='\r')
        if alt >= target_altitude * 0.95:
            print("\nReached target altitude.")
            break
        time.sleep(0.5)

def send_ned_velocity(vehicle, vx, vy, vz):
    """
    vx: north (m/s), vy: east (m/s), vz: down (m/s)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def stop_vehicle(vehicle):
    send_ned_velocity(vehicle, 0, 0, 0)

def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

# ========== CAMERA & QR DETECTION ==========

def camera_thread(cap):
    global latest_frame
    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        with frame_lock:
            latest_frame = frame.copy()
        time.sleep(0.01)

def get_latest_frame():
    with frame_lock:
        if latest_frame is not None:
            return latest_frame.copy()
    return None

def detect_qr_code_opencv(frame, qr_detector):
    """
    Returns (center, data, bbox, area) or (None, None, None, None)
    """
    if frame is None:
        return None, None, None, None
    data, bbox, rectified = qr_detector.detectAndDecode(frame)
    if bbox is not None and data:
        pts = bbox[0].astype(int)
        x_center = int(np.mean(pts[:, 0]))
        y_center = int(np.mean(pts[:, 1]))
        area = cv2.contourArea(pts)
        # draw
        for i in range(4):
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1) % 4]), (0, 255, 0), 2)
        cv2.circle(frame, (x_center, y_center), 6, (0, 0, 255), -1)
        return (x_center, y_center), data, pts, area
    return None, None, None, None

def calculate_offset_movement(qr_center, frame_center, altitude, smoothing=True):
    """
    Converts pixel offset to NED velocity commands (north, east).
    Returns (vel_north, vel_east, offset_x, offset_y)
    """
    global smoothed_offset_x, smoothed_offset_y

    offset_x = qr_center[0] - frame_center[0]  
    offset_y = qr_center[1] - frame_center[1]  

    if smoothing:
        smoothed_offset_x = SMOOTHING_ALPHA * offset_x + (1 - SMOOTHING_ALPHA) * smoothed_offset_x
        smoothed_offset_y = SMOOTHING_ALPHA * offset_y + (1 - SMOOTHING_ALPHA) * smoothed_offset_y
        use_x = smoothed_offset_x
        use_y = smoothed_offset_y
    else:
        use_x = offset_x
        use_y = offset_y

    velocity_scale = BASE_VELOCITY_SCALE * max(1.0, altitude)
    vel_east = use_x * velocity_scale
    vel_north = -use_y * velocity_scale  

    vel_east = max(min(vel_east, MAX_HORIZONTAL_VELOCITY), -MAX_HORIZONTAL_VELOCITY)
    vel_north = max(min(vel_north, MAX_HORIZONTAL_VELOCITY), -MAX_HORIZONTAL_VELOCITY)

    return vel_north, vel_east, offset_x, offset_y

# ========== SEARCH PATTERN GENERATOR ==========

def generate_lawnmower_pattern(home_location, size, spacing, altitude):
    waypoints = []
    lat_deg_per_meter = 1.0 / 111320.0
    lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(home_location.lat)))
    start_lat = home_location.lat + (5 * lat_deg_per_meter)
    start_lon = home_location.lon + (5 * lon_deg_per_meter)
    num_lines = max(1, int(size / spacing))
    for i in range(num_lines):
        north_offset = i * spacing * lat_deg_per_meter
        if i % 2 == 0:
            wp_start = LocationGlobalRelative(start_lat + north_offset, start_lon, altitude)
            wp_end = LocationGlobalRelative(start_lat + north_offset, start_lon + (size * lon_deg_per_meter), altitude)
        else:
            wp_start = LocationGlobalRelative(start_lat + north_offset, start_lon + (size * lon_deg_per_meter), altitude)
            wp_end = LocationGlobalRelative(start_lat + north_offset, start_lon, altitude)
        waypoints.append(wp_start)
        waypoints.append(wp_end)
    return waypoints

# ========== MAIN MISSION ==========

def main():
    global smoothed_offset_x, smoothed_offset_y
    vehicle = None
    cap = None
    try:
        print("=== AUTONOMOUS QR LANDING (Fixed) ===")
        # QR detector
        qr_detector = cv2.QRCodeDetector()
        # Camera
        cap = cv2.VideoCapture(CAMERA_SOURCE)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        time.sleep(1.0)
        if not cap.isOpened():
            print("ERROR: Camera source cannot be opened.")
            return
        # Warm-up frame
        ret, frm = cap.read()
        if not ret:
            print("ERROR: Cannot read warm-up frame.")
            return
        frame_h, frame_w = frm.shape[:2]
        frame_center = (frame_w // 2, frame_h // 2)
        print("Camera ready:", frame_w, "x", frame_h)

        # Start camera thread
        cam_t = threading.Thread(target=camera_thread, args=(cap,), daemon=True)
        cam_t.start()

        # Connect vehicle
        vehicle = connect_vehicle()

        # Takeoff
        arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
        time.sleep(1.0)

        home_loc = vehicle.location.global_relative_frame
        print(f"Home: {home_loc.lat:.7f}, {home_loc.lon:.7f}")

        # Search phase
        print("Starting lawnmower search...")
        waypoints = generate_lawnmower_pattern(home_loc, SEARCH_AREA_SIZE, SCAN_SPACING, SEARCH_ALTITUDE)
        qr_found = False
        qr_data = None

        for idx, wp in enumerate(waypoints):
            if qr_found:
                break
            print(f"→ Waypoint {idx+1}/{len(waypoints)}")
            vehicle.simple_goto(wp)
            vehicle.groundspeed = SCAN_SPEED
            while get_distance_metres(vehicle.location.global_relative_frame, wp) > 1.5:
                frame = get_latest_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                center, data, bbox, area = detect_qr_code_opencv(frame, qr_detector)
                cv2.putText(frame, f"Scan WP {idx+1}/{len(waypoints)}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.putText(frame, f"Alt: {vehicle.location.global_relative_frame.alt:.1f}m", (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                if center and data:
                    print("QR detected:", data)
                    qr_found = True
                    qr_data = data
                    stop_vehicle(vehicle)
                    time.sleep(0.8)
                    break
                cv2.imshow("UAV Camera Feed", frame)
                if cv2.waitKey(1) == 27:
                    raise KeyboardInterrupt
                time.sleep(DETECTION_INTERVAL)

        if not qr_found:
            print("QR not found. Initiating safe LAND at current position.")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                frame = get_latest_frame()
                if frame is not None:
                    cv2.putText(frame, "QR NOT FOUND - LANDING", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                    cv2.imshow("UAV Camera Feed", frame)
                    cv2.waitKey(1)
                time.sleep(0.5)
            return

        print("Approaching and centering above QR...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(0.5)
        stop_vehicle(vehicle)
        time.sleep(0.3)

        centered = False
        approach_iter = 0
        max_approach = 400

        while not centered and approach_iter < max_approach:
            frame = get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                approach_iter += 1
                continue
            center, data, bbox, area = detect_qr_code_opencv(frame, qr_detector)
            alt = vehicle.location.global_relative_frame.alt
            if center and area and area > MIN_QR_SIZE * 0.2:
                vel_n, vel_e, off_x, off_y = calculate_offset_movement(center, frame_center, alt, smoothing=True)
                dist_px = math.hypot(off_x, off_y)
                if dist_px < PRECISION_CENTER_THRESHOLD:
                    print("Precision centered (lock).")
                    stop_vehicle(vehicle)
                    centered = True
                    break
                else:
                    send_ned_velocity(vehicle, vel_n, vel_e, 0)
                cv2.putText(frame, f"Approaching. Dist: {int(dist_px)} px", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                stop_vehicle(vehicle)
                cv2.putText(frame, "Reacquiring QR...", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
            cv2.imshow("UAV Camera Feed", frame)
            if cv2.waitKey(1) == 27:
                raise KeyboardInterrupt
            approach_iter += 1
            time.sleep(0.08)

        if not centered:
            print("Could not precisely center. Proceeding with cautious descent attempt.")
        else:
            print("Centered. Starting controlled descent with tracking.")

        lost_frames = 0
        while vehicle.location.global_relative_frame.alt > LANDING_ALTITUDE:
            frame = get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            center, data, bbox, area = detect_qr_code_opencv(frame, qr_detector)
            alt = vehicle.location.global_relative_frame.alt

            if center and area and area > MIN_QR_SIZE * 0.3:
                lost_frames = 0
                vel_n, vel_e, off_x, off_y = calculate_offset_movement(center, frame_center, alt, smoothing=True)
                dist_px = math.hypot(off_x, off_y)
                if dist_px < QR_CENTER_THRESHOLD:
                  
                    send_ned_velocity(vehicle, vel_n * 0.35, vel_e * 0.35, DESCENT_VELOCITY)
                    status = "DESCENDING - CENTERED"
                else:
                    send_ned_velocity(vehicle, vel_n, vel_e, 0)
                    status = "CORRECTING POSITION"
                cv2.putText(frame, f"{status} | Dist:{int(dist_px)}px | Alt:{alt:.1f}m", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            else:
                lost_frames += 1
                if lost_frames <= MAX_LOST_QR_FRAMES:
                    send_ned_velocity(vehicle, 0, 0, DESCENT_VELOCITY * 0.3)
                    cv2.putText(frame, f"QR LOST - SLOW DESC ({lost_frames})", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
                else:
                    stop_vehicle(vehicle)
                    cv2.putText(frame, "QR LOST - HOLDING. RETRIES...", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            if vehicle.location.global_relative_frame.alt <= 3.0:
                print("Altitude low (<3m). Forcing LAND fallback to be safe.")
                vehicle.mode = VehicleMode("LAND")
                break

            cv2.imshow("UAV Camera Feed", frame)
            if cv2.waitKey(1) == 27:
                raise KeyboardInterrupt
            time.sleep(0.12)

        if vehicle.armed:
            print("Final correction & landing sequence.")
            stop_vehicle(vehicle)
            time.sleep(0.3)
            for i in range(35):
                frame = get_latest_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                center, data, bbox, area = detect_qr_code_opencv(frame, qr_detector)
                alt = vehicle.location.global_relative_frame.alt
                if center and area and area > MIN_QR_SIZE * 0.4:
                    vel_n, vel_e, off_x, off_y = calculate_offset_movement(center, frame_center, alt, smoothing=True)
                    dist_px = math.hypot(off_x, off_y)
                    if dist_px < PRECISION_CENTER_THRESHOLD:
                        stop_vehicle(vehicle)
                        print("Final position locked.")
                        break
                    send_ned_velocity(vehicle, vel_n * 0.25, vel_e * 0.25, 0)
                    cv2.putText(frame, f"Final adjust Dist:{int(dist_px)}px", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                else:
                    cv2.putText(frame, "Final reacquire...", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
                cv2.imshow("UAV Camera Feed", frame)
                if cv2.waitKey(1) == 27:
                    raise KeyboardInterrupt
                time.sleep(0.12)

        # Issue LAND
        if vehicle.armed:
            print("Switching to LAND mode...")
            vehicle.mode = VehicleMode("LAND")

        while vehicle.armed:
            frame = get_latest_frame()
            if frame is not None:
                cv2.putText(frame, f"LANDING... Alt:{vehicle.location.global_relative_frame.alt:.1f}m", (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.imshow("UAV Camera Feed", frame)
                cv2.waitKey(1)
            time.sleep(0.5)

        print("Mission complete: vehicle disarmed & landed.")

    except KeyboardInterrupt:
        print("Mission aborted by user. Landing immediately.")
        try:
            if vehicle:
                vehicle.mode = VehicleMode("LAND")
        except:
            pass
    except Exception as e:
        print("ERROR:", str(e))
        traceback.print_exc()
        try:
            if vehicle:
                vehicle.mode = VehicleMode("LAND")
        except:
            pass
    finally:
        print("Cleanup...")
        stop_event.set()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        if vehicle:
            try:
                vehicle.close()
            except:
                pass
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
