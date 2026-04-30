"""
gesture_controller.py  ─  Stanford Pupper gesture-driven locomotion

Gestures → commands (from gesture_recognition.py on /gesture_command):
  "Open_Palm"          → "stop"   → STANDING
  "Closed_Fist"        → "walk"   → WALKING    (slow + stable)
  "Pointing_Up"        → "stand"  → STANDING
  "Thumb_Down"         → "sit"    → SITTING    (slow, safe descent)
  "Victory"            → "spin"   → SPINNING   (spin-in-place)

Changes in this version:
  - Sit trajectory is interpolated over more steps (slow + safe)
  - Walking uses longer step period and lower step height (less camera shake)
  - Spin-in-place: diagonal pairs counter-rotate around body center
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import scipy.optimize
from enum import Enum, auto

np.set_printoptions(precision=3, suppress=True)


# ═══════════════════════════════════════════════════════════════════════════════
#  Robot constants
# ═══════════════════════════════════════════════════════════════════════════════

# Pupper leg geometry (metres)
L1 = 0.0      # hip offset (abduction link)
L2 = 0.1      # upper leg
L3 = 0.1      # lower leg

# Body offsets for each leg in body frame  (x forward, y left, z up)
LEG_OFFSETS = {
    'RF': np.array([ 0.07, -0.035, 0.0]),
    'LF': np.array([ 0.07,  0.035, 0.0]),
    'RB': np.array([-0.07, -0.035, 0.0]),
    'LB': np.array([-0.07,  0.035, 0.0]),
}
LEG_ORDER = ['RF', 'LF', 'RB', 'LB']

# Joint name order expected by the controller topic
JOINT_NAMES = [
    'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
    'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
    'leg_back_r_1',  'leg_back_r_2',  'leg_back_r_3',
    'leg_back_l_1',  'leg_back_l_2',  'leg_back_l_3',
]


# ═══════════════════════════════════════════════════════════════════════════════
#  HTM helpers  (identical to Lab 3)
# ═══════════════════════════════════════════════════════════════════════════════

def rotation_x(a):
    return np.array([[1,0,0,0],[0,np.cos(a),-np.sin(a),0],
                     [0,np.sin(a),np.cos(a),0],[0,0,0,1]])

def rotation_y(a):
    return np.array([[np.cos(a),0,np.sin(a),0],[0,1,0,0],
                     [-np.sin(a),0,np.cos(a),0],[0,0,0,1]])

def rotation_z(a):
    return np.array([[np.cos(a),-np.sin(a),0,0],[np.sin(a),np.cos(a),0,0],
                     [0,0,1,0],[0,0,0,1]])

def translation(x, y, z):
    return np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])


# ═══════════════════════════════════════════════════════════════════════════════
#  Forward kinematics for one leg
# ═══════════════════════════════════════════════════════════════════════════════

def forward_kinematics_leg(theta, leg):
    """Return end-effector position in body frame for one leg."""
    t1, t2, t3 = theta
    off = LEG_OFFSETS[leg]

    T = (translation(*off)
         @ rotation_z(t1)
         @ translation(0, 0, -L2)
         @ rotation_y(t2)
         @ translation(L3, 0, 0)
         @ rotation_y(t3))
    return T[:3, 3]


def ik_error(theta, p_des, leg):
    p = forward_kinematics_leg(theta, leg)
    return np.sum((p - p_des) ** 2)


def inverse_kinematics(p_des, leg, theta0=None):
    if theta0 is None:
        theta0 = np.zeros(3)
    res = scipy.optimize.minimize(ik_error, theta0, args=(p_des, leg),
                                  method='L-BFGS-B',
                                  bounds=[(-np.pi, np.pi)] * 3,
                                  options={'ftol': 1e-8, 'maxiter': 200})
    return res.x


# ═══════════════════════════════════════════════════════════════════════════════
#  Gait caches
# ═══════════════════════════════════════════════════════════════════════════════

def _build_ee_trajectory(waypoints):
    """Linear-interpolate between waypoints; returns list of ee positions."""
    pts = []
    for i in range(len(waypoints)):
        p0 = waypoints[i]
        p1 = waypoints[(i + 1) % len(waypoints)]
        for u in np.linspace(0, 1, 5, endpoint=False):
            pts.append((1 - u) * p0 + u * p1)
    return pts


def _solve_cache(ee_seqs):
    """
    ee_seqs: list of 4 ee-trajectory lists (one per leg, same length).
    Returns numpy array of shape (N, 12).
    """
    N = len(ee_seqs[0])
    cache = np.zeros((N, 12))
    prev = [np.zeros(3)] * 4
    for i in range(N):
        for j, leg in enumerate(LEG_ORDER):
            sol = inverse_kinematics(ee_seqs[j][i], leg, theta0=prev[j])
            prev[j] = sol
            cache[i, j*3:(j+1)*3] = sol
    return cache


# ─── Stand pose ──────────────────────────────────────────────────────────────

STAND_Z   = -0.12   # body height in standing (metres, negative = down)
STAND_EE  = {leg: LEG_OFFSETS[leg] + np.array([0, 0, STAND_Z])
             for leg in LEG_ORDER}


def build_stand_angles():
    angles = np.zeros(12)
    for j, leg in enumerate(LEG_ORDER):
        angles[j*3:(j+1)*3] = inverse_kinematics(STAND_EE[leg], leg)
    return angles


# ─── Sit pose ────────────────────────────────────────────────────────────────

SIT_EE = {
    'RF': LEG_OFFSETS['RF'] + np.array([ 0.04,  0.0, -0.07]),
    'LF': LEG_OFFSETS['LF'] + np.array([ 0.04,  0.0, -0.07]),
    'RB': LEG_OFFSETS['RB'] + np.array([-0.02,  0.0, -0.10]),
    'LB': LEG_OFFSETS['LB'] + np.array([-0.02,  0.0, -0.10]),
}


def build_sit_cache(stand_angles):
    """
    Slow, linear trajectory from standing pose to sit pose.
    Uses many interpolation steps so the robot lowers gradually and safely.
    N_STEPS controls how slowly it sits — larger = slower/safer.
    """
    N_STEPS = 60   # at 10 Hz control = ~6 seconds to fully sit  ← SLOW & SAFE
    cache = np.zeros((N_STEPS, 12))

    # Compute sit joint angles
    sit_angles = np.zeros(12)
    for j, leg in enumerate(LEG_ORDER):
        sit_angles[j*3:(j+1)*3] = inverse_kinematics(SIT_EE[leg], leg)

    for i in range(N_STEPS):
        u = i / (N_STEPS - 1)                    # 0 → 1
        # Ease-in-out: slow at start, slow at end — gentle on servos
        u_smooth = 0.5 - 0.5 * np.cos(np.pi * u)
        cache[i] = (1 - u_smooth) * stand_angles + u_smooth * sit_angles

    return cache, sit_angles


# ─── Trot gait ───────────────────────────────────────────────────────────────
# Slowed down significantly: longer stance, lower step height, more interp pts
# so the body moves smoothly and the camera doesn't shake.

STEP_HEIGHT  = 0.022   # was ~0.04 — lower swing = less body rock
STEP_REACH   = 0.025   # fore-aft reach per step
GAIT_Z       = STAND_Z  # stay at stand height during gait


def _trot_ee_seqs(direction=1.0):
    """
    Build 4 ee trajectories for a trot gait.
    direction=+1 → forward,  direction=-1 → backward
    Each swing arc: 8 waypoints; each stance: 8 waypoints → 16 pts × 5 interp = 80 per cycle
    Diagonal pairs (RF+LB) and (LF+RB) are 180° out of phase.
    """
    reach = STEP_REACH * direction

    def swing(base):
        # Lift up and forward
        return [
            base + np.array([ reach, 0, 0]),          # liftoff
            base + np.array([ reach*0.5, 0,  STEP_HEIGHT * 0.5]),
            base + np.array([ 0,          0,  STEP_HEIGHT]),  # peak
            base + np.array([-reach*0.5, 0,  STEP_HEIGHT * 0.5]),
            base + np.array([-reach,     0,  0]),      # touch-down
        ]

    def stance(base):
        # Slide backward to push body forward
        return [
            base + np.array([-reach,      0, 0]),
            base + np.array([-reach*0.5,  0, 0]),
            base + np.array([  0,          0, 0]),
            base + np.array([ reach*0.5,   0, 0]),
            base + np.array([ reach,        0, 0]),
        ]

    seqs = []
    for j, leg in enumerate(LEG_ORDER):
        base = STAND_EE[leg]
        # RF(0) and LB(3) swing together; LF(1) and RB(2) swing together
        if leg in ('RF', 'LB'):
            ee = swing(base) + stance(base)
        else:
            ee = stance(base) + swing(base)
        seqs.append(ee)
    return seqs


def build_trot_cache(direction=1.0):
    seqs = _trot_ee_seqs(direction)
    traj = [_build_ee_trajectory(s) for s in seqs]
    return _solve_cache(traj)


# ─── Spin-in-place gait ──────────────────────────────────────────────────────
# Body stays centred.  Front feet step rightward, rear feet step leftward
# (or vice-versa) — net effect is yaw rotation about the body's vertical axis.
#
# Implementation: same trot-style alternating diagonal pairs, but the
# "forward" direction of each leg is set to the tangential direction around
# the body's centre, not body-x.  We approximate this as:
#   RF → step left  (+y)     LF → step right (-y)     (during swing)
#   RB → step right (-y)     LB → step left  (+y)
# This counter-rotates the two sides and produces CW yaw.
# Flip the signs to get CCW.

SPIN_REACH = 0.020   # lateral reach per step


def build_spin_cache():
    """Spin clockwise (from above). Diagonal pairs alternate swing/stance."""

    def swing_lat(base, dy):
        return [
            base + np.array([0,  dy,        0]),
            base + np.array([0,  dy * 0.5,  STEP_HEIGHT * 0.5]),
            base + np.array([0,  0,          STEP_HEIGHT]),
            base + np.array([0, -dy * 0.5,  STEP_HEIGHT * 0.5]),
            base + np.array([0, -dy,         0]),
        ]

    def stance_lat(base, dy):
        # During stance, slide opposite direction (ground reaction)
        return [
            base + np.array([0, -dy,         0]),
            base + np.array([0, -dy * 0.5,   0]),
            base + np.array([0,  0,           0]),
            base + np.array([0,  dy * 0.5,    0]),
            base + np.array([0,  dy,          0]),
        ]

    # CW yaw:  RF steps left (+y), LF steps right (-y),
    #          RB steps right (-y), LB steps left (+y)
    spin_dirs = {'RF': +SPIN_REACH, 'LF': -SPIN_REACH,
                 'RB': -SPIN_REACH, 'LB': +SPIN_REACH}

    seqs = []
    for leg in LEG_ORDER:
        base = STAND_EE[leg]
        dy   = spin_dirs[leg]
        # RF+LB swing together (as in trot)
        if leg in ('RF', 'LB'):
            ee = swing_lat(base, dy) + stance_lat(base, dy)
        else:
            ee = stance_lat(base, dy) + swing_lat(base, dy)
        seqs.append(ee)

    traj = [_build_ee_trajectory(s) for s in seqs]
    return _solve_cache(traj)


# ═══════════════════════════════════════════════════════════════════════════════
#  State machine
# ═══════════════════════════════════════════════════════════════════════════════

class RobotState(Enum):
    STANDING = auto()
    SITTING  = auto()
    WALKING  = auto()
    SPINNING = auto()


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ═══════════════════════════════════════════════════════════════════════════════

class GestureControllerNode(Node):

    # ── Control-loop frequencies ──────────────────────────────────────────────
    # Lower = smoother / less shake.  PD at 10 Hz, IK already runs offline.
    PD_HZ = 10   # was 20 — half the rate = half the micro-jitter

    def __init__(self):
        super().__init__('gesture_controller')
        self.get_logger().info("Gesture Controller starting — building gait caches…")

        # Pubs / subs
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_command_controller/commands', 10)
        self.create_subscription(
            String, '/gesture_command', self._gesture_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10)

        # State
        self.state              = RobotState.STANDING
        self.counter            = 0
        self.joint_positions    = None
        self.joint_velocities   = None
        self._walk_timer        = None

        # Pre-build all caches (slow once at startup, fast during execution)
        self.get_logger().info("  Building stand pose…")
        self.stand_angles = build_stand_angles()

        self.get_logger().info("  Building sit cache…")
        self.sit_cache, self.sit_angles = build_sit_cache(self.stand_angles)

        self.get_logger().info("  Building trot cache (forward)…")
        self.trot_fwd_cache = build_trot_cache(direction=+1.0)

        self.get_logger().info("  Building spin cache…")
        self.spin_cache = build_spin_cache()

        self.target_joint_positions = self.stand_angles.copy()
        self.get_logger().info("All caches ready.  Waiting for gestures…")

        # Timers
        self.create_timer(1.0 / self.PD_HZ, self._ik_callback)
        self.create_timer(1.0 / self.PD_HZ, self._pd_callback)

    # ── Gesture callback ──────────────────────────────────────────────────────

    def _cancel_walk_timer(self):
        if self._walk_timer is not None:
            self._walk_timer.cancel()
            self._walk_timer = None

    def _gesture_callback(self, msg: String):
        self._cancel_walk_timer()
        cmd = msg.data.strip().lower()

        if cmd == "stop":
            self._transition_to(RobotState.STANDING, "stop → STANDING")

        elif cmd == "walk":
            if self.state == RobotState.SITTING:
                self._transition_to(RobotState.STANDING, "walk from sit → STANDING first")
                self._walk_timer = self.create_timer(2.0, self._delayed_walk_fwd)
            else:
                self._transition_to(RobotState.WALKING, "walk → WALKING")

        elif cmd == "sit":
            self._transition_to(RobotState.SITTING, "sit → SITTING")

        elif cmd == "stand":
            self._transition_to(RobotState.STANDING, "stand → STANDING")

        elif cmd == "spin":
            if self.state == RobotState.SITTING:
                self.get_logger().warn("Cannot spin from SITTING — stand first.")
            else:
                self._transition_to(RobotState.SPINNING, "spin → SPINNING")

    def _delayed_walk_fwd(self):
        self._cancel_walk_timer()
        if self.state == RobotState.STANDING:
            self._transition_to(RobotState.WALKING, "delayed walk after stand")

    def _transition_to(self, new_state: RobotState, reason: str):
        self.get_logger().info(f"State transition: {self.state.name} → {new_state.name}  [{reason}]")
        self.state   = new_state
        self.counter = 0

    # ── IK / command selection ────────────────────────────────────────────────

    def _ik_callback(self):
        if self.state == RobotState.WALKING:
            cache = self.trot_fwd_cache
            self.target_joint_positions = cache[self.counter % len(cache)]
            self.counter += 1

        elif self.state == RobotState.SPINNING:
            cache = self.spin_cache
            self.target_joint_positions = cache[self.counter % len(cache)]
            self.counter += 1

        elif self.state == RobotState.SITTING:
            if self.counter < len(self.sit_cache):
                # Still descending — step through sit trajectory one frame at a time
                self.target_joint_positions = self.sit_cache[self.counter]
                self.counter += 1
            else:
                # Reached full sit — hold there
                self.target_joint_positions = self.sit_angles.copy()

        else:  # STANDING
            self.target_joint_positions = self.stand_angles.copy()

    # ── PD publish ────────────────────────────────────────────────────────────

    def _pd_callback(self):
        msg      = Float64MultiArray()
        msg.data = self.target_joint_positions.tolist()
        self.cmd_pub.publish(msg)

    # ── Joint state subscriber ────────────────────────────────────────────────

    def _joint_callback(self, msg: JointState):
        self.get_logger().info("Joint states received!", once=True)
        try:
            self.joint_positions  = np.array(
                [msg.position[msg.name.index(j)] for j in JOINT_NAMES])
            self.joint_velocities = np.array(
                [msg.velocity[msg.name.index(j)] for j in JOINT_NAMES])
        except ValueError:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = GestureControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down gesture controller …")
    finally:
        stand_msg      = Float64MultiArray()
        stand_msg.data = node.stand_angles.tolist()
        node.cmd_pub.publish(stand_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
