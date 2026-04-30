import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import scipy.optimize
from enum import Enum, auto

np.set_printoptions(precision=3, suppress=True)


# ═══════════════════════════════════════════════════════════════════════════════
#  HTM helpers 
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

    swing_phase = np.array([touch_down, mid_swing, liftoff,
                             stand_3,   stand_2,   stand_1])
    stand_phase = np.array([stand_3, stand_2, stand_1,
                             touch_down, mid_swing, liftoff])

    return [
        swing_phase + rf_off,
        stand_phase + lf_off,
        stand_phase + rb_off,
        swing_phase + lb_off,
    ]


def build_sit_waypoints():
    """
    Dog-sit transition: front legs extend forward while back legs tuck under.
    Front legs reach x=0.10 (further forward) and hold z=-0.14 (fully extended).
    Back legs rise from z=-0.14 to z=-0.06 and shift forward under the hip.
    """
    rf_off = np.array([ 0.06, -0.09, 0])
    lf_off = np.array([ 0.06,  0.09, 0])
    rb_off = np.array([-0.11, -0.09, 0])
    lb_off = np.array([-0.11,  0.09, 0])

    # Front legs: extend forward (x: 0 → 0.04) and stay fully extended (z=-0.14)
    front_sit = np.array([
        [ 0.00, 0, -0.14],
        [ 0.01, 0, -0.14],
        [ 0.02, 0, -0.14],
        [ 0.03, 0, -0.14],
        [ 0.04, 0, -0.14],
        [ 0.04, 0, -0.14],
    ])

    # Back legs: tuck upward and shift foot forward under the hip
    back_sit = np.array([
        [ 0.00, 0, -0.14],
        [ 0.01, 0, -0.12],
        [ 0.02, 0, -0.10],
        [ 0.03, 0, -0.08],
        [ 0.04, 0, -0.06],
        [ 0.04, 0, -0.06],
    ])

    return [front_sit + rf_off, front_sit + lf_off,
            back_sit  + rb_off, back_sit  + lb_off]


def build_spin_waypoints():
    """
    Approximate 360° spin by shifting feet laterally in opposite directions
    for diagonal pairs — creates body yaw.
    """
    rf_off = np.array([ 0.06, -0.09, 0])
    lf_off = np.array([ 0.06,  0.09, 0])
    rb_off = np.array([-0.11, -0.09, 0])
    lb_off = np.array([-0.11,  0.09, 0])

    swing_spin = np.array([
        [ 0,    0.03, -0.14],
        [ 0,    0.03, -0.05],
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


# Static hold poses
STAND_EE_TARGETS = [
    np.array([ 0.06, -0.09, -0.14]),   # RF
    np.array([ 0.06,  0.09, -0.14]),   # LF
    np.array([-0.11, -0.09, -0.14]),   # RB
    np.array([-0.11,  0.09, -0.14]),   # LB
]

SIT_EE_TARGETS = [
    np.array([ 0.10, -0.09, -0.14]),   # RF: extended forward
    np.array([ 0.10,  0.09, -0.14]),   # LF: extended forward
    np.array([-0.07, -0.09, -0.06]),   # RB: tucked up and shifted forward
    np.array([-0.07,  0.09, -0.06]),   # LB: tucked up and shifted forward
]


# ═══════════════════════════════════════════════════════════════════════════════
#  State machine
# ═══════════════════════════════════════════════════════════════════════════════

class RobotState(Enum):
    STANDING = auto()
    WALKING  = auto()
    SITTING  = auto()
    SPINNING = auto()
    TRANSITIONING = auto()


# ═══════════════════════════════════════════════════════════════════════════════
#  Main node
# ═══════════════════════════════════════════════════════════════════════════════

class GestureControllerNode(Node):

    def __init__(self):
        super().__init__("gesture_controller")

        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self._joint_callback, 10)

        self.gesture_sub = self.create_subscription(
            String, "/gesture_command", self._gesture_callback, 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/forward_command_controller/commands", 10)

        self.joint_positions  = None
        self.joint_velocities = None
        self.state = RobotState.STANDING

        self.get_logger().info("Building gait caches (this may take ~10 s) …")

        self.trot_cache  = self._build_cache(build_trot_waypoints())
        self.spin_cache  = self._build_cache(build_spin_waypoints())
        self.sit_cache   = self._build_cache(build_sit_waypoints(), n_steps=30)

        self.stand_angles = self._compute_static_pose(STAND_EE_TARGETS)
        self.sit_angles   = self._compute_static_pose(SIT_EE_TARGETS)

        self.get_logger().info("Gait caches built.")

        self.counter = 0
        self.active_cache = self.trot_cache
        self.target_joint_positions = self.stand_angles.copy()
        self._active_gait_name = "standing"

        self.pd_timer = self.create_timer(1.0 / 200.0, self._pd_callback)
        self.ik_timer = self.create_timer(1.0 / 100.0, self._ik_callback)

    # ─────────────────────────────────────────────────────────────────────────
    #  FK
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
        angles = []
        guess = [0.0, 0.0, 0.0]
        for i, ee in enumerate(ee_targets):
            q = self._ik_single_leg(ee, i, guess)
            angles.append(q)
            guess = q.tolist()
        return np.concatenate(angles)

    # ─────────────────────────────────────────────────────────────────────────
    #  Gait cache builder
    # ─────────────────────────────────────────────────────────────────────────

    def _interpolate(self, t, waypoints):
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
        return np.concatenate(cache, axis=1)

    # ─────────────────────────────────────────────────────────────────────────
    #  Gesture callback
    # ─────────────────────────────────────────────────────────────────────────

    def _cancel_walk_timer(self):
        if hasattr(self, '_walk_timer') and self._walk_timer is not None:
            self._walk_timer.cancel()
            self._walk_timer = None

    def _gesture_callback(self, msg: String):
        self._cancel_walk_timer()
        cmd = msg.data.strip().lower()

        if cmd == "stop":
            self._transition_to(RobotState.STANDING, "stop → STANDING")

        elif cmd == "walk":
            if self.state == RobotState.SITTING:
                self._transition_to(RobotState.STANDING, "walk (from sit) → STANDING first")
                self._walk_timer = self.create_timer(1.5, self._delayed_walk)
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

    def _transition_to(self, new_state: RobotState, log_msg: str):
        self.get_logger().info(f"State transition: {self.state.name} → {new_state.name}  [{log_msg}]")
        self.state   = new_state
        self.counter = 0

    def _delayed_walk(self):
        self._cancel_walk_timer()
        if self.state == RobotState.STANDING:
            self._transition_to(RobotState.WALKING, "delayed walk after stand")

    # ─────────────────────────────────────────────────────────────────────────
    #  Timer callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _ik_callback(self):
        if self.state == RobotState.WALKING:
            cache = self.trot_cache
        elif self.state == RobotState.SPINNING:
            cache = self.spin_cache
        elif self.state == RobotState.SITTING:
            if self.counter < len(self.sit_cache):
                cache = self.sit_cache
            else:
                self.target_joint_positions = self.sit_angles.copy()
                return
        else:
            self.target_joint_positions = self.stand_angles.copy()
            return

        self.target_joint_positions = cache[self.counter % len(cache)]
        self.counter += 1

    def _pd_callback(self):
        msg = Float64MultiArray()
        msg.data = self.target_joint_positions.tolist()
        self.cmd_pub.publish(msg)

    def _joint_callback(self, msg: JointState):
        self.get_logger().info("Joint states received!", once=True)
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
