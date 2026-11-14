import tkinter as tk
from tkinter import ttk, scrolledtext
import math
import asyncio
import threading
import queue
import time
from bleak import BleakScanner, BleakClient

# ==============================================================================
# BLE CONFIGURATION & UUIDS (Nordic UART Service)
# ==============================================================================
SERVICE_UUID           = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

INITIAL_FRAME = "<90,0,0,45,30>"
BLE_SEND_INTERVAL = 0.050  # 20 Hz transmission rate (50 ms)

# Mechanical Link Dimensions (cm)
H_BASE = 5.0
L1 = 6.1
L2 = 6.8
L3 = 2.5
WRIST_OFFSET_DEG = 98.13

# ==============================================================================
# KINEMATICS ENGINE (FK & IK)
# ==============================================================================
def calculate_fk(theta_base, theta_shoulder, theta_elbow, theta_wrist):
    """Computes (X, Y, Z) and radial reach (r) of Joint 5 in cm."""
    rad_base     = math.radians(90.0 - theta_base)
    rad_shoulder = math.radians(theta_shoulder)
    rad_elbow    = math.radians(theta_elbow)
    rad_wrist    = math.radians(theta_wrist)

    diff_elbow_shoulder = rad_elbow - rad_shoulder
    phi3 = diff_elbow_shoulder + rad_wrist - math.radians(WRIST_OFFSET_DEG)

    r = (L1 * math.sin(rad_shoulder) + 
         L2 * math.cos(diff_elbow_shoulder) + 
         L3 * math.cos(phi3))

    x = r * math.sin(rad_base)
    y = r * math.cos(rad_base)
    z = (H_BASE + 
         L1 * math.cos(rad_shoulder) + 
         L2 * math.sin(diff_elbow_shoulder) + 
         L3 * math.sin(phi3))

    return round(x, 2), round(y, 2), round(z, 2), round(r, 2)

def solve_ik(x, y, z, phi3_deg=-53.13):
    """
    Computes Inverse Kinematics for Joint 5.
    Returns: (theta_base, theta_shoulder, theta_elbow, theta_wrist) in degrees.
    """
    r = math.hypot(x, y)
    theta_base = 90.0 - math.degrees(math.atan2(x, y))

    phi3_rad = math.radians(phi3_deg)
    r4 = r - L3 * math.cos(phi3_rad)
    z4 = (z - H_BASE) - L3 * math.sin(phi3_rad)

    D = math.hypot(r4, z4)
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError(f"Unreachable: Distance D={D:.2f}cm outside [{abs(L1-L2):.2f}, {L1+L2:.2f}]")

    psi = math.atan2(r4, z4)
    cos_alpha = (L1**2 + D**2 - L2**2) / (2.0 * L1 * D)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    rad_theta1 = psi - alpha
    theta1 = math.degrees(rad_theta1)

    diff_elbow_shoulder = math.atan2(z4 - L1 * math.cos(rad_theta1),
                                     r4 - L1 * math.sin(rad_theta1))
    theta2 = theta1 + math.degrees(diff_elbow_shoulder)
    theta3 = phi3_deg - math.degrees(diff_elbow_shoulder) + WRIST_OFFSET_DEG

    # Hardware boundary clamps
    if not (-5 <= theta_base <= 185 and -5 <= theta1 <= 95 and -5 <= theta2 <= 95 and -5 <= theta3 <= 95):
        raise ValueError("Calculated angles exceed servo physical limits.")

    b = max(0.0, min(180.0, theta_base))
    s = max(0.0, min(90.0, theta1))
    e = max(0.0, min(90.0, theta2))
    w = max(0.0, min(90.0, theta3))

    return round(b, 2), round(s, 2), round(e, 2), round(w, 2)

# ==============================================================================
# MAIN GUI APPLICATION
# ==============================================================================
class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control Panel — Inverse Kinematics & Trajectories")
        self.root.geometry("940x720")
        self.root.resizable(False, False)

        # Thread synchronization
        self.log_queue = queue.Queue()
        self.command_lock = threading.Lock()
        self.latest_command = INITIAL_FRAME
        self.stop_event = threading.Event()
        self.ble_connected = False
        self.is_internal_update = False  # Prevents recursion between FK and IK sliders

        # Current High-Precision Float Angles
        self.current_angles = [90.0, 0.0, 0.0, 45.0, 30.0]

        # Joint Variables
        self.var_base     = tk.IntVar(value=90)
        self.var_shoulder = tk.IntVar(value=0)
        self.var_elbow    = tk.IntVar(value=0)
        self.var_wrist    = tk.IntVar(value=45)
        self.var_grab     = tk.IntVar(value=30)
        self.slider_vars  = [self.var_base, self.var_shoulder, self.var_elbow, self.var_wrist, self.var_grab]
        self.slider_labels = []

        # Cartesian IK Variables (cm)
        self.var_x = tk.DoubleVar(value=0.0)
        self.var_y = tk.DoubleVar(value=8.3)
        self.var_z = tk.DoubleVar(value=9.1)

        # Motion & Speed Parameters
        self.var_speed = tk.DoubleVar(value=25.0)  # Speed in °/s
        self.var_dwell = tk.DoubleVar(value=0.5)   # Pause at waypoint (s)
        self.var_loop  = tk.BooleanVar(value=False)

        # Motion State Engine
        self.is_interpolating = False
        self.is_sequence_running = False
        self.motion_after_id = None
        self.sequence_step_index = 0
        self.active_trajectory = []
        self.sequence_keys = ["Preset 1 (Init)", "Preset 2 (Home)", "Preset 3 (Reach)", 
                              "Preset 4 (Clamp)", "Preset 5 (Rest)"]

        # 5 Customizable Presets (Storing joint angles)
        self.presets = {
            "Preset 1 (Init)":  [90, 0,  0,  45, 30],
            "Preset 2 (Home)":  [90, 45, 45, 45, 30],
            "Preset 3 (Reach)": [90, 60, 30, 45, 0],
            "Preset 4 (Clamp)": [90, 0,  0,  45, 60],
            "Preset 5 (Rest)":  [90, 0,  0,  0,  0]
        }

        # Build UI layout
        self.setup_styles()
        self.build_ui()

        # Start BLE Worker Thread
        self.ble_thread = threading.Thread(target=self.run_ble_loop, daemon=True)
        self.ble_thread.start()

        # Start Log Consumer
        self.root.after(50, self.process_log_queue)

        # Synchronize Cartesian Sliders on Launch
        self.sync_cartesian_from_joints()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Status.TLabel", font=("Segoe UI", 9, "bold"))
        style.configure("Accent.TButton", font=("Segoe UI", 9, "bold"))

    def build_ui(self):
        # Top Bar: Connection & Active Command Frame
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)

        ttk.Label(top_frame, text="BLE Status: ", style="Header.TLabel").pack(side=tk.LEFT)
        self.lbl_status = ttk.Label(top_frame, text="Searching for RobotArm_C3...", foreground="#d9534f", style="Status.TLabel")
        self.lbl_status.pack(side=tk.LEFT, padx=5)

        self.lbl_cmd = ttk.Label(top_frame, text=f"Active Frame: {INITIAL_FRAME}", font=("Consolas", 10))
        self.lbl_cmd.pack(side=tk.RIGHT)

        # Main Body Split
        body = ttk.Frame(self.root, padding=10)
        body.pack(fill=tk.BOTH, expand=True)

        left_col = ttk.LabelFrame(body, text=" Joint Angle Actuation (FK Controls) ", padding=12)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        right_col = ttk.Frame(body)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(6, 0))

        # ----------------- LEFT: 5 SERVO SLIDERS -----------------
        self.create_joint_slider(left_col, "Servo 1 — Base (Yaw)", self.var_base, 0, 180)
        self.create_joint_slider(left_col, "Servo 2 — Shoulder (Pitch)", self.var_shoulder, 0, 90)
        self.create_joint_slider(left_col, "Servo 3 — Elbow (Pitch)", self.var_elbow, 0, 90)
        self.create_joint_slider(left_col, "Servo 4 — Wrist (Pitch)", self.var_wrist, 0, 90)
        self.create_joint_slider(left_col, "Servo 5 — Gripper (Jaws)", self.var_grab, 0, 60)

        # ----------------- RIGHT: TOP (CARTESIAN IK SLIDERS) -----------------
        ik_frame = ttk.LabelFrame(right_col, text=" Inverse Kinematics Control (Joint 5 Tip) ", padding=10)
        ik_frame.pack(fill=tk.X, pady=(0, 6))

        self.lbl_x_val = self.create_cartesian_slider(ik_frame, "Target X (Left/Right):", self.var_x, -12.0, 12.0)
        self.lbl_y_val = self.create_cartesian_slider(ik_frame, "Target Y (Forward):", self.var_y, 4.0, 15.0)
        self.lbl_z_val = self.create_cartesian_slider(ik_frame, "Target Z (Height):", self.var_z, 2.0, 16.0)

        # ----------------- RIGHT: MIDDLE (MOTION ENGINE & PRESETS) -----------------
        motion_frame = ttk.LabelFrame(right_col, text=" Continuous Motion, Circle Trajectory & Presets ", padding=8)
        motion_frame.pack(fill=tk.X, pady=(0, 6))

        # Speed & Dwell Sliders
        speed_box = ttk.Frame(motion_frame)
        speed_box.pack(fill=tk.X, pady=(0, 3))
        ttk.Label(speed_box, text="Speed (°/s):", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        self.lbl_speed_val = ttk.Label(speed_box, text=f"{int(self.var_speed.get())}°/s", font=("Consolas", 9))
        self.lbl_speed_val.pack(side=tk.RIGHT)
        ttk.Scale(speed_box, from_=5.0, to=80.0, variable=self.var_speed, orient=tk.HORIZONTAL,
                  command=lambda v: self.lbl_speed_val.config(text=f"{int(float(v))}°/s")).pack(fill=tk.X, padx=8)

        dwell_box = ttk.Frame(motion_frame)
        dwell_box.pack(fill=tk.X, pady=(2, 5))
        ttk.Label(dwell_box, text="Dwell Pause:", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.lbl_dwell_val = ttk.Label(dwell_box, text=f"{self.var_dwell.get():.1f}s", font=("Consolas", 9))
        self.lbl_dwell_val.pack(side=tk.RIGHT)
        ttk.Scale(dwell_box, from_=0.0, to=3.0, variable=self.var_dwell, orient=tk.HORIZONTAL,
                  command=lambda v: self.lbl_dwell_val.config(text=f"{float(v):.1f}s")).pack(fill=tk.X, padx=8)

        # Preset Buttons
        p_row = ttk.Frame(motion_frame)
        p_row.pack(fill=tk.X, pady=(2, 4))
        for name in self.presets.keys():
            short_name = name.split()[1].strip("()")
            btn = ttk.Button(p_row, text=short_name, width=7,
                             command=lambda n=name: self.on_preset_click(n))
            btn.pack(side=tk.LEFT, padx=2, expand=True)

        # Execution Controls & Circle Generator
        seq_row = ttk.Frame(motion_frame)
        seq_row.pack(fill=tk.X, pady=(3, 3))

        self.btn_seq = ttk.Button(seq_row, text="▶ Start Sequence", style="Accent.TButton", command=self.toggle_sequence)
        self.btn_seq.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        self.btn_circle = ttk.Button(seq_row, text="⭕ Draw Circle (XZ)", style="Accent.TButton", command=self.start_circle_trajectory)
        self.btn_circle.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        self.btn_stop = ttk.Button(seq_row, text="⏹ Stop", command=self.stop_motion)
        self.btn_stop.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        ttk.Checkbutton(seq_row, text="Loop", variable=self.var_loop).pack(side=tk.LEFT, padx=4)

        # Save Preset Controls
        save_row = ttk.Frame(motion_frame)
        save_row.pack(fill=tk.X, pady=(3, 0))
        ttk.Label(save_row, text="Save angles to:", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)
        self.preset_combo = ttk.Combobox(save_row, values=list(self.presets.keys()), state="readonly", width=14)
        self.preset_combo.current(0)
        self.preset_combo.pack(side=tk.LEFT, padx=3)
        ttk.Button(save_row, text="Save", width=6, command=self.save_preset).pack(side=tk.LEFT, padx=2)

        # ----------------- RIGHT: BOTTOM (OUTPUT TERMINAL) -----------------
        term_frame = ttk.LabelFrame(right_col, text=" Output Terminal & Telemetry ", padding=6)
        term_frame.pack(fill=tk.BOTH, expand=True)

        self.terminal = scrolledtext.ScrolledText(term_frame, wrap=tk.WORD, height=7,
                                                  font=("Consolas", 9), bg="#1e1e1e", fg="#d4d4d4",
                                                  insertbackground="white")
        self.terminal.pack(fill=tk.BOTH, expand=True)

    def create_joint_slider(self, parent, label_text, var, from_val, to_val):
        box = ttk.Frame(parent, padding=4)
        box.pack(fill=tk.X, pady=3)
        header = ttk.Frame(box)
        header.pack(fill=tk.X)
        ttk.Label(header, text=label_text).pack(side=tk.LEFT)
        val_lbl = ttk.Label(header, text=f"{var.get()}°", font=("Consolas", 10, "bold"), foreground="#007acc")
        val_lbl.pack(side=tk.RIGHT)
        self.slider_labels.append(val_lbl)

        slider = ttk.Scale(box, from_=from_val, to=to_val, variable=var, orient=tk.HORIZONTAL,
                           command=lambda val: self.on_joint_slider_drag(var, val_lbl))
        slider.pack(fill=tk.X, pady=(2, 0))

    def create_cartesian_slider(self, parent, label_text, var, from_val, to_val):
        box = ttk.Frame(parent, padding=2)
        box.pack(fill=tk.X, pady=2)
        header = ttk.Frame(box)
        header.pack(fill=tk.X)
        ttk.Label(header, text=label_text, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        val_lbl = ttk.Label(header, text=f"{var.get():+.2f} cm", font=("Consolas", 9, "bold"), foreground="#005fb8")
        val_lbl.pack(side=tk.RIGHT)

        slider = ttk.Scale(box, from_=from_val, to=to_val, variable=var, orient=tk.HORIZONTAL,
                           command=lambda val: self.on_cartesian_slider_drag(var, val_lbl))
        slider.pack(fill=tk.X, pady=(1, 0))
        return val_lbl

    # ==========================================================================
    # BI-DIRECTIONAL KINEMATIC EVENT HANDLERS
    # ==========================================================================
    def on_joint_slider_drag(self, var, label):
        if self.is_internal_update:
            return

        if self.is_interpolating or self.is_sequence_running:
            self.stop_motion()

        val = int(round(var.get()))
        var.set(val)
        label.config(text=f"{val}°")

        # Sync float memory
        for idx in range(5):
            self.current_angles[idx] = float(self.slider_vars[idx].get())

        self.sync_cartesian_from_joints()
        self.queue_ble_frame()

    def sync_cartesian_from_joints(self):
        """Forward Kinematics: updates Cartesian sliders based on joint values."""
        self.is_internal_update = True
        b = self.var_base.get()
        s = self.var_shoulder.get()
        e = self.var_elbow.get()
        w = self.var_wrist.get()

        x, y, z, _ = calculate_fk(b, s, e, w)
        self.var_x.set(x)
        self.var_y.set(y)
        self.var_z.set(z)

        self.lbl_x_val.config(text=f"{x:+.2f} cm")
        self.lbl_y_val.config(text=f"{y:+.2f} cm")
        self.lbl_z_val.config(text=f"{z:+.2f} cm")
        self.is_internal_update = False

    def on_cartesian_slider_drag(self, var, label):
        if self.is_internal_update:
            return

        if self.is_interpolating or self.is_sequence_running:
            self.stop_motion()

        label.config(text=f"{var.get():+.2f} cm")
        self.solve_and_apply_ik()

    def solve_and_apply_ik(self):
        """Inverse Kinematics: computes angles from Cartesian sliders and updates joints."""
        x = self.var_x.get()
        y = self.var_y.get()
        z = self.var_z.get()

        try:
            b, s, e, w = solve_ik(x, y, z)
            self.is_internal_update = True

            # Update Joint Sliders
            self.var_base.set(int(round(b)))
            self.var_shoulder.set(int(round(s)))
            self.var_elbow.set(int(round(e)))
            self.var_wrist.set(int(round(w)))

            for i, val in enumerate([b, s, e, w, self.var_grab.get()]):
                self.current_angles[i] = float(val)
                self.slider_labels[i].config(text=f"{int(round(val))}°")

            self.is_internal_update = False
            self.queue_ble_frame()

        except ValueError as err:
            self.log(f"IK Warning: {err}")

    def queue_ble_frame(self):
        frame = f"<{self.var_base.get()},{self.var_shoulder.get()},{self.var_elbow.get()},{self.var_wrist.get()},{self.var_grab.get()}>"
        self.lbl_cmd.config(text=f"Active Frame: {frame}")
        with self.command_lock:
            self.latest_command = frame

    # ==========================================================================
    # CIRCLE TRAJECTORY GENERATOR (XZ Plane, Y=10cm, Center=(0, 8.5), R=2cm)
    # ==========================================================================
    def start_circle_trajectory(self):
        """Generates waypoints and executes circle in XZ plane via Inverse Kinematics."""
        self.stop_motion()
        self.log("--- Generating Circle Trajectory (Y=10.0cm, Center=(0, 8.5), R=2.0cm) ---")

        radius = 2.0
        center_x = 0.0
        center_z = 8.5
        fixed_y = 10.0
        num_points = 48  # Discretized waypoints

        waypoints = []
        for i in range(num_points + 1):
            alpha = 2 * math.pi * (i / num_points)
            px = center_x + radius * math.cos(alpha)
            py = fixed_y
            pz = center_z + radius * math.sin(alpha)

            try:
                b, s, e, w = solve_ik(px, py, pz)
                waypoints.append([b, s, e, w, self.var_grab.get()])
            except ValueError as e:
                self.log(f"Circle generation error at alpha={math.degrees(alpha):.0f}°: {e}")
                return

        self.active_trajectory = waypoints
        self.sequence_step_index = 0
        self.is_sequence_running = True
        self.btn_seq.config(text="⏸ Pause Sequence")
        self.log(f"Starting circle execution ({len(waypoints)} waypoints)...")
        self.run_next_trajectory_point()

    def run_next_trajectory_point(self):
        if not self.is_sequence_running:
            return

        if self.sequence_step_index >= len(self.active_trajectory):
            if self.var_loop.get():
                self.sequence_step_index = 0
                self.log("Looping circle trajectory...")
            else:
                self.stop_motion()
                self.log("Circle trajectory completed successfully.")
                return

        target = self.active_trajectory[self.sequence_step_index]
        self.sequence_step_index += 1
        self.start_interpolation(target, on_complete_callback=self.on_trajectory_point_reached)

    def on_trajectory_point_reached(self):
        if not self.is_sequence_running:
            return
        # Continuous motion without dwell for smooth circles
        self.run_next_trajectory_point()

    # ==========================================================================
    # PRESET TRAJECTORY SEQUENCING
    # ==========================================================================
    def on_preset_click(self, name):
        if self.is_sequence_running:
            self.stop_motion()
        target = self.presets[name]
        self.log(f"Gliding to {name} at {int(self.var_speed.get())}°/s...")
        self.start_interpolation(target)

    def toggle_sequence(self):
        if self.is_sequence_running:
            self.stop_motion()
        else:
            self.active_trajectory = [self.presets[k] for k in self.sequence_keys]
            self.is_sequence_running = True
            self.btn_seq.config(text="⏸ Pause Sequence")
            self.sequence_step_index = 0
            self.log("--- Starting Posture Sequence ---")
            self.run_next_sequence_waypoint()

    def run_next_sequence_waypoint(self):
        if not self.is_sequence_running:
            return

        if self.sequence_step_index >= len(self.active_trajectory):
            if self.var_loop.get():
                self.sequence_step_index = 0
                self.log("Looping sequence...")
            else:
                self.stop_motion()
                self.log("Sequence completed.")
                return

        target = self.active_trajectory[self.sequence_step_index]
        preset_name = self.sequence_keys[self.sequence_step_index]
        self.log(f"Waypoint [{self.sequence_step_index + 1}/{len(self.sequence_keys)}]: {preset_name}")
        self.sequence_step_index += 1
        self.start_interpolation(target, on_complete_callback=self.on_sequence_waypoint_reached)

    def on_sequence_waypoint_reached(self):
        if not self.is_sequence_running:
            return
        dwell_ms = int(self.var_dwell.get() * 1000)
        if dwell_ms > 0:
            self.motion_after_id = self.root.after(dwell_ms, self.run_next_sequence_waypoint)
        else:
            self.run_next_sequence_waypoint()

    # ==========================================================================
    # LINEAR INTERPOLATION (LERP) STEPPER
    # ==========================================================================
    def start_interpolation(self, target_angles, on_complete_callback=None):
        if self.motion_after_id:
            self.root.after_cancel(self.motion_after_id)
            self.motion_after_id = None

        self.is_interpolating = True
        self.target_angles = [float(a) for a in target_angles]
        self.on_complete_callback = on_complete_callback
        self.motion_step()

    def motion_step(self):
        if not self.is_interpolating:
            return

        dt = 0.050
        speed = max(1.0, self.var_speed.get())
        max_step = speed * dt

        deltas = [self.target_angles[i] - self.current_angles[i] for i in range(5)]
        max_delta = max(abs(d) for d in deltas)

        if max_delta <= max_step or max_delta < 0.001:
            for i in range(5):
                self.current_angles[i] = self.target_angles[i]
                self.slider_vars[i].set(int(round(self.target_angles[i])))
                self.slider_labels[i].config(text=f"{int(round(self.target_angles[i]))}°")

            self.sync_cartesian_from_joints()
            self.queue_ble_frame()
            self.is_interpolating = False
            self.motion_after_id = None

            if self.on_complete_callback:
                self.on_complete_callback()
            return

        fraction = max_step / max_delta
        for i in range(5):
            self.current_angles[i] += deltas[i] * fraction
            int_val = int(round(self.current_angles[i]))
            self.slider_vars[i].set(int_val)
            self.slider_labels[i].config(text=f"{int_val}°")

        self.sync_cartesian_from_joints()
        self.queue_ble_frame()
        self.motion_after_id = self.root.after(50, self.motion_step)

    def stop_motion(self):
        if self.motion_after_id:
            self.root.after_cancel(self.motion_after_id)
            self.motion_after_id = None

        self.is_interpolating = False
        self.is_sequence_running = False
        self.btn_seq.config(text="▶ Start Sequence")
        self.log("Motion stopped.")

    def save_preset(self):
        target = self.preset_combo.get()
        current = [self.var_base.get(), self.var_shoulder.get(), self.var_elbow.get(),
                   self.var_wrist.get(), self.var_grab.get()]
        self.presets[target] = current
        self.log(f"Saved state to {target}: <{','.join(map(str, current))}>")

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")

    def process_log_queue(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get_nowait()
            self.terminal.insert(tk.END, msg + "\n")
            self.terminal.see(tk.END)
        self.root.after(50, self.process_log_queue)

    # ==========================================================================
    # BACKGROUND BLE WORKER
    # ==========================================================================
    def run_ble_loop(self):
        asyncio.run(self.ble_worker())

    async def ble_worker(self):
        self.log("Scanning for 'RobotArm_C3'...")
        device = await BleakScanner.find_device_by_name("RobotArm_C3", timeout=10.0)

        if not device:
            self.log("ERROR: 'RobotArm_C3' not found. Check power & press RST.")
            self.root.after(0, lambda: self.lbl_status.config(text="Disconnected (Not Found)", foreground="#d9534f"))
            return

        self.log(f"Found {device.name} [{device.address}]. Connecting...")

        def on_telemetry(sender, data):
            msg = data.decode("utf-8", errors="ignore").strip()
            self.log(f"STM32 Feedback ◄ {msg}")

        try:
            async with BleakClient(device) as client:
                self.ble_connected = True
                self.log("BLE Link Established (Nordic UART Service Connected)")
                self.root.after(0, lambda: self.lbl_status.config(text="Connected (Live)", foreground="#5cb85c"))

                await client.start_notify(CHARACTERISTIC_UUID_TX, on_telemetry)

                # Send initial frame
                await client.write_gatt_char(CHARACTERISTIC_UUID_RX, INITIAL_FRAME.encode(), response=False)
                self.log(f"TX Initial Frame ► {INITIAL_FRAME}")

                last_sent = INITIAL_FRAME
                while not self.stop_event.is_set():
                    with self.command_lock:
                        curr = self.latest_command

                    if curr != last_sent:
                        await client.write_gatt_char(CHARACTERISTIC_UUID_RX, curr.encode(), response=False)
                        last_sent = curr

                    await asyncio.sleep(BLE_SEND_INTERVAL)

        except Exception as e:
            self.log(f"BLE Exception: {str(e)}")
            self.root.after(0, lambda: self.lbl_status.config(text="Disconnected (Error)", foreground="#d9534f"))

# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.stop_event.set(), root.destroy()))
    root.mainloop()