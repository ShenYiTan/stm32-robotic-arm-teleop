import cv2
import mediapipe as mp
import numpy as np
import time
import math
import asyncio
import threading
from bleak import BleakScanner, BleakClient

# ==============================================================================
# BLE CONFIGURATION & UUIDS (Nordic UART Service)
# ==============================================================================
SERVICE_UUID           = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

INITIAL_FRAME = "<90,0,0,45,30>"
BLE_SEND_INTERVAL = 0.050  # 20 Hz streaming (every 50 ms)

# Thread-safe synchronization primitives
latest_command = INITIAL_FRAME
command_lock = threading.Lock()
stop_event = threading.Event()
ble_connected_event = threading.Event()

# ==============================================================================
# CALIBRATION CONSTANTS & WINDOW DIMENSIONS (cm)
# ==============================================================================
SCREEN_WIDTH_CM   = 14.3
SCREEN_HEIGHT_CM  = 10.7

ELBOW_DIST_MIN_CM = 0.6   # Closer to camera -> 0 deg (as configured)
ELBOW_DIST_MAX_CM = 1.2   # Further from camera -> 90 deg (as configured)

WRIST_DIST_MIN_CM = 0.4   # Fingers closed -> 90 deg (as configured)
WRIST_DIST_MAX_CM = 2.2   # Fingers spread -> 0 deg (as configured)

# ==============================================================================
# MEDIAPIPE TASKS CONFIGURATION
# ==============================================================================
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),         # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),         # Index finger
    (5, 9), (9, 10), (10, 11), (11, 12),    # Middle finger
    (9, 13), (13, 14), (14, 15), (15, 16),  # Ring finger
    (13, 17), (17, 18), (18, 19), (19, 20), # Pinky
    (0, 17)                                 # Wrist / palm base
]

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="models/hand_landmarker.task"),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=2,  # Configured to track 2 hands for safety interlock
    min_hand_detection_confidence=0.6,
    min_hand_presence_confidence=0.6,
    min_tracking_confidence=0.6
)

# ==============================================================================
# FILTER CLASS (EMA + DEADBAND + RESET)
# ==============================================================================
class FilteredJoint:
    def __init__(self, min_val, max_val, default_val, alpha=0.25, deadband=2.0):
        self.min_val = min_val
        self.max_val = max_val
        self.default_val = float(default_val)
        self.current_val = float(default_val)
        self.alpha = alpha         # Smoothing factor (0.0 to 1.0)
        self.deadband = deadband   # 2 degree threshold

    def update(self, raw_value):
        clamped = max(self.min_val, min(self.max_val, raw_value))
        smoothed = (self.alpha * clamped) + ((1.0 - self.alpha) * self.current_val)
        
        if abs(smoothed - self.current_val) >= self.deadband:
            self.current_val = smoothed
            
        return int(round(self.current_val))

    def reset_to_default(self):
        self.current_val = self.default_val
        return int(round(self.current_val))

# Initialize joints with hardware limits and default postures
joint_base     = FilteredJoint(min_val=0, max_val=180, default_val=90) # Base: 0-180 deg[cite: 6]
joint_shoulder = FilteredJoint(min_val=0, max_val=90,  default_val=0)  # Shoulder: 0-90 deg[cite: 6]
joint_elbow    = FilteredJoint(min_val=0, max_val=90,  default_val=0)  # Elbow: 0-90 deg[cite: 6]
joint_wrist    = FilteredJoint(min_val=0, max_val=90,  default_val=45) # Wrist: 0-90 deg[cite: 6]
joint_grab     = FilteredJoint(min_val=0, max_val=60,  default_val=30) # Grab: 0-60 deg[cite: 6]

def reset_all_joints():
    joint_base.reset_to_default()
    joint_shoulder.reset_to_default()
    joint_elbow.reset_to_default()
    joint_wrist.reset_to_default()
    joint_grab.reset_to_default()

def calculate_cm_distance(ptA, ptB):
    dx_cm = (ptA.x - ptB.x) * SCREEN_WIDTH_CM
    dy_cm = (ptA.y - ptB.y) * SCREEN_HEIGHT_CM
    return math.hypot(dx_cm, dy_cm)

# ==============================================================================
# BACKGROUND BLE WORKER THREAD
# ==============================================================================
async def ble_async_loop():
    global latest_command
    print("Searching for 'RobotArm_C3'...")
    device = await BleakScanner.find_device_by_name("RobotArm_C3", timeout=10.0)

    if not device:
        print("Error: 'RobotArm_C3' not found. Ensure ESP32 is powered and RST was pressed.")
        stop_event.set()
        return

    print(f"Connecting to {device.name} ({device.address})...")
    async with BleakClient(device) as client:
        print(f"BLE Connected: {client.is_connected}")
        
        # Send initial default frame
        await client.write_gatt_char(CHARACTERISTIC_UUID_RX, INITIAL_FRAME.encode(), response=False)
        ble_connected_event.set()

        last_sent_frame = ""
        while not stop_event.is_set():
            with command_lock:
                current_frame = latest_command

            if current_frame != last_sent_frame:
                try:
                    await client.write_gatt_char(CHARACTERISTIC_UUID_RX, current_frame.encode(), response=False)
                    last_sent_frame = current_frame
                except Exception as e:
                    print(f"BLE Write Error: {e}")

            await asyncio.sleep(BLE_SEND_INTERVAL)

def run_ble_thread():
    asyncio.run(ble_async_loop())

# ==============================================================================
# MAIN FUNCTION (OpenCV + 3-Second Pre-Activation Safety Machine)
# ==============================================================================
def main():
    global latest_command

    # Start background BLE thread
    ble_thread = threading.Thread(target=run_ble_thread, daemon=True)
    ble_thread.start()

    print("Waiting for BLE link to establish...")
    if not ble_connected_event.wait(timeout=10.0):
        if stop_event.is_set():
            return
        print("Warning: BLE connection timed out. Opening vision feed in offline mode...")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not access webcam.")
        stop_event.set()
        return

    print("\n--- SAFETY CONTROLLER READY ---")
    print("1. Start with TWO hands or NO hands to stay in INITIAL state.")
    print("2. Present ONE hand: 3-second countdown initiates before active control.")
    print("3. Present TWO hands: Immediately snaps back to INITIAL state.")
    print("4. Press 'q' at any time to exit.\n")

    # Safety State Tracking Variables
    control_active = False
    countdown_start_time = None
    zero_hand_start_time = None
    COUNTDOWN_DURATION = 3.0

    with HandLandmarker.create_from_options(options) as landmarker:
        while cap.isOpened() and not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            timestamp_ms = int(time.time() * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            num_detected_hands = len(result.hand_landmarks) if result.hand_landmarks else 0
            current_time = time.time()

            # Render landmarks for all detected hands
            if result.hand_landmarks:
                for hand in result.hand_landmarks:
                    coords = [(int(lm.x * w), int(lm.y * h)) for lm in hand]
                    for start_idx, end_idx in HAND_CONNECTIONS:
                        cv2.line(frame, coords[start_idx], coords[end_idx], (0, 255, 0), 2)
                    for pt in coords:
                        cv2.circle(frame, pt, 4, (0, 0, 255), -1)

            # Draw reference box [0.3 to 0.7]
            cv2.rectangle(frame, (int(0.3 * w), int(0.3 * h)), (int(0.7 * w), int(0.7 * h)), (255, 255, 0), 1)

            # ==================================================================
            # SAFETY STATE MACHINE LOGIC
            # ==================================================================

            # ------------------------------------------------------------------
            # CASE 1: TWO (OR MORE) HANDS DETECTED -> IMMEDIATE INITIAL STATE
            # ------------------------------------------------------------------
            if num_detected_hands >= 2:
                control_active = False
                countdown_start_time = None
                zero_hand_start_time = None
                reset_all_joints()

                command_frame = INITIAL_FRAME
                with command_lock:
                    latest_command = command_frame

                cv2.rectangle(frame, (10, 10), (520, 95), (150, 100, 0), -1)
                cv2.putText(frame, "STANDBY: 2 HANDS DETECTED", (20, 42),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"ARM AT INIT: {command_frame}", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(frame, "Remove one hand to begin 3s countdown", (20, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

            # ------------------------------------------------------------------
            # CASE 2: EXACTLY ONE HAND DETECTED
            # ------------------------------------------------------------------
            elif num_detected_hands == 1:
                zero_hand_start_time = None  # Reset 0-hand timer

                # SUB-CASE 2A: COUNTDOWN PHASE (Not active yet)
                if not control_active:
                    if countdown_start_time is None:
                        countdown_start_time = current_time

                    elapsed = current_time - countdown_start_time
                    remaining = COUNTDOWN_DURATION - elapsed

                    if remaining > 0:
                        # Keep arm locked at initial frame during countdown
                        reset_all_joints()
                        command_frame = INITIAL_FRAME
                        with command_lock:
                            latest_command = command_frame

                        # Countdown Display (Orange/Yellow Alert)
                        cv2.rectangle(frame, (10, 10), (520, 95), (0, 140, 255), -1)
                        cv2.putText(frame, f"PREPARING: STARTING IN {remaining:.1f}s", (20, 42),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        cv2.putText(frame, f"ARM HELD AT: {command_frame}", (20, 65),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                        cv2.putText(frame, "Hold hand steady inside target box...", (20, 85),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
                    else:
                        # 3 seconds elapsed: Activate live tracking
                        control_active = True
                        countdown_start_time = None

                # SUB-CASE 2B: ACTIVE CONTROL PHASE (After countdown completes)
                if control_active:
                    hand = result.hand_landmarks[0]
                    coords = [(int(lm.x * w), int(lm.y * h)) for lm in hand]

                    wrist_pt   = hand[0]
                    thumb_cmc  = hand[1]
                    index_tip  = hand[8]
                    middle_tip = hand[12]

                    # 1. Base (0-180 deg)
                    raw_base = np.interp(wrist_pt.x, [0.3, 0.7], [180, 0])

                    # 2. Shoulder (0-90 deg)
                    raw_shoulder = np.interp(wrist_pt.y, [0.3, 0.7], [0, 90])

                    # 3. Elbow (0-90 deg)
                    d_elbow_cm = calculate_cm_distance(wrist_pt, thumb_cmc)
                    raw_elbow = np.interp(d_elbow_cm, [ELBOW_DIST_MIN_CM, ELBOW_DIST_MAX_CM], [0, 90])

                    # 4. Wrist (0-90 deg)
                    d_wrist_cm = calculate_cm_distance(index_tip, middle_tip)
                    raw_wrist = np.interp(d_wrist_cm, [WRIST_DIST_MIN_CM, WRIST_DIST_MAX_CM], [90, 0])

                    # 5. Grab (0-60 deg)
                    palm_scale = math.hypot(coords[0][0] - coords[9][0], coords[0][1] - coords[9][1])
                    pinch_px = math.hypot(coords[4][0] - coords[8][0], coords[4][1] - coords[8][1])
                    if palm_scale > 0:
                        norm_pinch = pinch_px / palm_scale
                        raw_grab = np.interp(norm_pinch, [0.20, 0.70], [0, 60])
                    else:
                        raw_grab = joint_grab.current_val

                    # Apply smoothing and bounds
                    b     = joint_base.update(raw_base)
                    s     = joint_shoulder.update(raw_shoulder)
                    e     = joint_elbow.update(raw_elbow)
                    w_deg = joint_wrist.update(raw_wrist)
                    g     = joint_grab.update(raw_grab)

                    command_frame = f"<{b},{s},{e},{w_deg},{g}>"
                    with command_lock:
                        latest_command = command_frame

                    # Active Green HUD
                    cv2.rectangle(frame, (10, 10), (490, 95), (0, 0, 0), -1)
                    cv2.putText(frame, f"ACTIVE: {command_frame}", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
                    cv2.putText(frame, f"B:{b} S:{s} E:{e} W:{w_deg} G:{g}", (20, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                    cv2.putText(frame, f"Elbow: {d_elbow_cm:.2f}cm | Wrist: {d_wrist_cm:.2f}cm", (20, 85),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 220, 255), 1)

            # ------------------------------------------------------------------
            # CASE 3: ZERO HANDS DETECTED -> REVERT TO INITIAL STATE
            # ------------------------------------------------------------------
            else:
                if zero_hand_start_time is None:
                    zero_hand_start_time = current_time

                # Allow 0.3s tolerance for camera frame drops before resetting
                if (current_time - zero_hand_start_time) > 0.3:
                    control_active = False
                    countdown_start_time = None
                    reset_all_joints()

                    command_frame = INITIAL_FRAME
                    with command_lock:
                        latest_command = command_frame

                    cv2.rectangle(frame, (10, 10), (490, 80), (50, 50, 50), -1)
                    cv2.putText(frame, "NO HAND DETECTED (INIT STATE)", (20, 42),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 200, 200), 2)
                    cv2.putText(frame, f"ARM AT INIT: {command_frame}", (20, 68),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1)

            cv2.imshow("Hand Gesture Arm Control", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

    cap.release()
    cv2.destroyAllWindows()
    stop_event.set()
    ble_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()