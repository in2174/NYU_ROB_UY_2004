"""
gesture_controller.py

ROS2 node that:
  1. Subscribes to /gesture_command (String) from gesture_recognition.py
  2. Runs a state machine: STANDING → WALKING / SITTING / SPINNING / STOPPING
  3. Drives all 4 legs using the IK + gait cache from Lab 3

State machine:
  "stop"  → STANDING  (safe default – stabilises into neutral stand pose)
  "walk"  → WALKING   (trot gait, loops until interrupted)
  "sit"   → SITTING   (lower body, hold)
  "stand" → STANDING  (rise to neutral stand pose)
  "spin"  → SPINNING  (360° turn: alternating swing of diagonal pairs)

Run alongside gesture_recognition.py:
  ros2 run <pkg> gesture_recognition &
  ros2 run <pkg> gesture_controller
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
#  Gait waypoint builder
# ═══════════════════════════════════════════════════════════════════════════════

def build_trot_waypoints():
    """Return (ee_triangle_positions list, offsets list) — same as Lab 3."""
    touch_down   = np.array([-0.05,  0,    -0.14])
    stand_1      = np.array([-0.025, 0,    -0.14])
    stand_2      = np.array([ 0,     0,    -0.14])
    stand_3      = np.array([ 0.025, 0,    -0.14])
    liftoff      = np.array([ 0.05,  0,    -0.14])
    mid_swing    = np.array([ 0,     0,    -0.05])

    rf_off = np.array([ 0.06, -0.09, 0])
    lf_off = np.array([ 0.06,  0.09, 0])
    rb_off = np.array([-0.11, -0.09, 0])
    lb_off = np.array([-0.11,  0.09, 0])

    # Diagonal pairs in phase: RF+LB swing together, LF+RB swing together
    swing_phase = np.array([touch_down, mid_swing, liftoff,
                             stand_3,   stand_2,   stand_1])
    stand_phase = np.array([stand_3, stand_2, stand_1,
                             touch_down, mid_swing, liftoff])

    return [
        swing_phase + rf_off,   # RF
        stand_phase + lf_off,   # LF
        stand_phase + rb_off,   # RB
        swing_phase + lb_off,   # LB
    ]


def build_sit_waypoints():
    """
    Dog-sit transition: front legs stay extended straight down while
    back legs tuck up underneath the body.

    Front legs (RF, LF): z stays at -0.14 throughout — no movement.
    Back legs (RB, LB):  z rises from -0.14 → -0.06 (tucked under),
                         x shifts slightly forward (+0.04) so the hips
                         drop naturally, mimicking a real dog sit.

    6 waypoints used to give a smooth interpolation through the transition.
    """
    rf_off = np.array([ 0.06, -0.09, 0])
    lf_off = np.array([ 0.06,  0.09, 0])
    rb_off = np.array([-0.11, -0.09, 0])
    lb_off = np.array([-0.11,  0.09, 0])

    # Front legs: stay perfectly still (all 6 waypoints identical)
    front_sit = np.array([
        [0, 0, -0.14],
        [0, 0, -0.14],
        [0, 0, -0.14],
        [0, 0, -0.14],
        [0, 0, -0.14],
        [0, 0, -0.14],
    ])

    # Back legs: tuck upward and shift foot forward under the hip
    # Step 1-4: gradually raise z and advance x; step 5-6: hold final sit
    back_sit = np.array([
        [ 0.00, 0, -0.14],   # start: fully extended
        [ 0.01, 0, -0.12],   # beginning to tuck
        [ 0.02, 0, -0.10],
        [ 0.03, 0, -0.08],
        [ 0.04, 0, -0.06],   # fully tucked
        [ 0.04, 0, -0.06],   # hold
    ])

    return [front_sit + rf_off, front_sit + lf_off,
            back_sit  + rb_off, back_sit  + lb_off]


def build_spin_waypoints():
    """
    Approximate 360° spin by shifting feet laterally in opposite directions
    for diagonal pairs — creates body yaw.
    Positive y = left, negative y = right.
    """
    rf_off = np.array([ 0.06, -0.09, 0])
    lf_off = np.array([ 0.06,  0.09, 0])
    rb_off = np.array([-0.11, -0.09, 0])
    lb_off = np.array([-0.11,  0.09, 0])

    # RF and LB swing outward (+y), LF and RB swing inward (-y)
    # This biases the body to rotate clockwise viewed from above
    swing_spin = np.array([
        [ 0,    0.03, -0.14],
        [ 0,    0.03, -0.05],   # mid swing
        [ 0,    0.03, -0.14],
        [ 0,   -0.03, -0.14],
        [ 0,   -0.03, -0.14],
        [ 0,   -0.03, -0.14],
    ])
    stand_spin = np.array([
        [ 0,   -0.03, -0.14],
        [ 0,   -0.03, -0.14],
        [ 0,   -0.03, -0.14],
        [ 0,    0.03, -0.14],
        [ 0,    0.03, -0.05],
        [ 0,    0.03, -0.14],
    ])

    return [swing_spin + rf_off, stand_spin + lf_off,
            stand_spin + rb_off, swing_spin + lb_off]


# Static hold poses (single joint-angle arrays, 12 DOF)
STAND_EE_TARGETS = [
    np.array([ 0.06, -0.09, -0.14]),   # RF
    np.array([ 0.06,  0.09, -0.14]),   # LF
    np.array([-0.11, -0.09, -0.14]),   # RB
    np.array([-0.11,  0.09, -0.14]),   # LB
]

SIT_EE_TARGETS = [
    np.array([ 0.06,  -0.09, -0.14]),   # RF: straight down (unchanged)
    np.array([ 0.06,   0.09, -0.14]),   # LF: straight down (unchanged)
    np.array([-0.07,  -0.09, -0.06]),   # RB: tucked up and shifted forward
    np.array([-0.07,   0.09, -0.06]),   # LB: tucked up and shifted forward
]


# ═══════════════════════════════════════════════════════════════════════════════
#  State machine
# ═══════════════════════════════════════════════════════════════════════════════

class RobotState(Enum):
    STANDING = auto()
    WALKING  = auto()
    SITTING  = auto()
    SPINNING = auto()
    TRANSITIONING = auto()   # brief state while switching gaits


# ═══════════════════════════════════════════════════════════════════════════════
#  Main node
# ═══════════════════════════════════════════════════════════════════════════════

class GestureControllerNode(Node):

    def __init__(self):
        super().__init__("gesture_controller")

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self._joint_callback, 10)

        self.gesture_sub = self.create_subscription(
            String, "/gesture_command", self._gesture_callback, 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/forward_command_controller/commands", 10)

        # ── Robot state ───────────────────────────────────────────────────────
        self.joint_positions  = None
        self.joint_velocities = None
        self.state = RobotState.STANDING

        # ── Gait caches ───────────────────────────────────────────────────────
        self.get_logger().info("Building gait caches (this may take ~10 s) …")

        self.trot_cache  = self._build_cache(build_trot_waypoints())
        self.spin_cache  = self._build_cache(build_spin_waypoints())
        self.sit_cache   = self._build_cache(build_sit_waypoints(), n_steps=30)

        self.stand_angles = self._compute_static_pose(STAND_EE_TARGETS)
        self.sit_angles   = self._compute_static_pose(SIT_EE_TARGETS)

        self.get_logger().info("Gait caches built.")

        # ── Playback ──────────────────────────────────────────────────────────
        self.counter = 0
        self.active_cache = self.trot_cache   # will be overridden by state
        self.target_joint_positions = self.stand_angles.copy()

        # Current active gait name (for logging)
        self._active_gait_name = "standing"

        # ── Timers ────────────────────────────────────────────────────────────
        self.pd_timer = self.create_timer(1.0 / 200.0, self._pd_callback)
        self.ik_timer = self.create_timer(1.0 / 100.0, self._ik_callback)

    # ─────────────────────────────────────────────────────────────────────────
    #  FK  (identical to Lab 3)
    # ─────────────────────────────────────────────────────────────────────────

    def _fr_fk(self, t):
        T = (translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(t[0])
             @ rotation_y(-1.57080) @ rotation_z(t[1])
             @ translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(t[2])
             @ translation(0.06231, -0.06216, 0.01800))
        return T[:3, 3]

    def _fl_fk(self, t):
        T = (translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-t[0])
             @ rotation_y(-1.57080) @ rotation_z(t[1])
             @ translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-t[2])
             @ translation(0.06231, -0.06216, -0.01800))
        return T[:3, 3]

    def _br_fk(self, t):
        T = (translation(-0.07500, -0.07250, 0) @ rotation_x(1.57080) @ rotation_z(t[0])
             @ rotation_y(-1.57080) @ rotation_z(t[1])
             @ translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(t[2])
             @ translation(0.06231, -0.06216, 0.01800))
        return T[:3, 3]

    def _bl_fk(self, t):
        T = (translation(-0.07500, 0.07250, 0) @ rotation_x(1.57080) @ rotation_z(-t[0])
             @ rotation_y(-1.57080) @ rotation_z(t[1])
             @ translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-t[2])
             @ translation(0.06231, -0.06216, -0.01800))
        return T[:3, 3]

    @property
    def _fk_fns(self):
        return [self._fr_fk, self._fl_fk, self._br_fk, self._bl_fk]

    # ─────────────────────────────────────────────────────────────────────────
    #  IK helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _ik_single_leg(self, target_ee, leg_index, guess=None):
        if guess is None:
            guess = [0.0, 0.0, 0.0]
        fk = self._fk_fns[leg_index]
        def cost(theta):
            err = fk(theta) - target_ee
            return err @ err
        return scipy.optimize.minimize(cost, guess).x

    def _compute_static_pose(self, ee_targets):
        """Solve IK for a static 4-leg pose, return 12-DOF angle array."""
        angles = []
        guess = [0.0, 0.0, 0.0]
        for i, ee in enumerate(ee_targets):
            q = self._ik_single_leg(ee, i, guess)
            angles.append(q)
            guess = q.tolist()
        return np.concatenate(angles)

    # ─────────────────────────────────────────────────────────────────────────
    #  Gait cache builder  (same pattern as Lab 3)
    # ─────────────────────────────────────────────────────────────────────────

    def _interpolate(self, t, waypoints):
        """Linearly interpolate along the 6-waypoint loop. t ∈ [0,1)."""
        pts = waypoints
        n   = pts.shape[0]
        t   = t % 1.0
        s   = t * n
        i   = int(s)
        u   = s - i
        p0  = pts[i]
        p1  = pts[(i + 1) % n]
        return (1 - u) * p0 + u * p1

    def _build_cache(self, ee_waypoints_per_leg, n_steps=50):
        """
        Pre-compute IK for every time step and every leg.
        Returns ndarray of shape (n_steps, 12).
        """
        cache = []
        for leg_idx in range(4):
            leg_cache = []
            guess = [0.0, 0.0, 0.0]
            for t in np.linspace(0, 1, n_steps, endpoint=False):
                ee  = self._interpolate(t, ee_waypoints_per_leg[leg_idx])
                q   = self._ik_single_leg(ee, leg_idx, guess)
                leg_cache.append(q)
                guess = q.tolist()
            cache.append(leg_cache)

        # (4, n_steps, 3) → (n_steps, 12)
        return np.concatenate(cache, axis=1)

    # ─────────────────────────────────────────────────────────────────────────
    #  Gesture callback  →  state transitions
    # ─────────────────────────────────────────────────────────────────────────

    def _gesture_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        prev_state = self.state
        
        if cmd == "stop":
            self._transition_to(RobotState.STANDING, "stop → STANDING")

        elif cmd == "walk":
            if self.state == RobotState.SITTING:
                # Must stand first, then walk
                self._transition_to(RobotState.STANDING, "walk (from sit) → STANDING first")
                # Schedule walk after one gait cycle — simplest safe approach
                self.create_timer(1.5, self._delayed_walk, cancel_on_shutdown=True)
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

        else:
            self.get_logger().warn(f"Unknown gesture command: '{cmd}'")

    def _transition_to(self, new_state: RobotState, log_msg: str):
        self.get_logger().info(f"State transition: {self.state.name} → {new_state.name}  [{log_msg}]")
        self.state   = new_state
        self.counter = 0   # restart from beginning of gait cycle on every transition

    def _delayed_walk(self):
        """Called after stand has had time to complete; switch to WALKING."""
        self._transition_to(RobotState.WALKING, "delayed walk after stand")
        # One-shot timer: destroy it immediately (can't auto-cancel in ROS2 Humble easily)

    # ─────────────────────────────────────────────────────────────────────────
    #  Timer callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _ik_callback(self):
        """Advance gait counter and update target joint positions at 100 Hz."""
        if self.joint_positions is None:
            return

        if self.state == RobotState.WALKING:
            cache = self.trot_cache
        elif self.state == RobotState.SPINNING:
            cache = self.spin_cache
        elif self.state == RobotState.SITTING:
            # Play sit transition once, then hold final pose
            if self.counter < len(self.sit_cache):
                cache = self.sit_cache
            else:
                self.target_joint_positions = self.sit_angles.copy()
                return
        else:
            # STANDING — hold static pose, no gait cycling
            self.target_joint_positions = self.stand_angles.copy()
            return

        self.target_joint_positions = cache[self.counter % len(cache)]
        self.counter += 1

    def _pd_callback(self):
        """Publish target joint positions to position controller at 200 Hz."""
        if self.target_joint_positions is None:
            return
        msg = Float64MultiArray()
        msg.data = self.target_joint_positions.tolist()
        self.cmd_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    #  Joint state listener
    # ─────────────────────────────────────────────────────────────────────────

    def _joint_callback(self, msg: JointState):
        joints = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
            'leg_back_r_1',  'leg_back_r_2',  'leg_back_r_3',
            'leg_back_l_1',  'leg_back_l_2',  'leg_back_l_3',
        ]
        self.joint_positions  = np.array(
            [msg.position[msg.name.index(j)] for j in joints])
        self.joint_velocities = np.array(
            [msg.velocity[msg.name.index(j)] for j in joints])


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
        # Safe shutdown: send standing pose, then zeros
        if node.joint_positions is not None:
            stand_msg = Float64MultiArray()
            stand_msg.data = node.stand_angles.tolist()
            node.cmd_pub.publish(stand_msg)

        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0] * 12
        node.cmd_pub.publish(zero_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
