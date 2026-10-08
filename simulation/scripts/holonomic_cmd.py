"""
CoppeliaSim 4.7 Python child script: drives a holonomic (mecanum) base from a
geometry_msgs/msg/TwistStamped topic using rclpy.

    twist.linear.x  -> vx (forward/backward)
    twist.linear.y  -> vy (left/right)
    twist.angular.z -> wz (yaw rate)

It also publishes the measured twist of the base (same fields and units, in
the base frame) on MEASURED_TWIST_TOPIC after every simulation step.

Attach it as a non-threaded Python child script to the robot's base object
(the object whose tree contains the wheel joints), or set ROBOT_BASE_PATH.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
TWIST_TOPIC = '/sas_b1/b1_1/set/holonomic_target_twist'
MEASURED_TWIST_TOPIC = '/sas_b1/b1_1/get/holonomic_twist'
NODE_NAME = 'coppeliasim_holonomic_base'

# '..' = the parent of this script object, i.e. the object the script is
# attached to (in CoppeliaSim 4.7, '.' is the script object itself).
# Use an absolute path such as '/UnitreeB1' to attach the script elsewhere.
ROBOT_BASE_PATH = '..'

# Order matters: front-left, rear-left, rear-right, front-right.
WHEEL_JOINT_NAMES = ['rollingJoint_fl', 'rollingJoint_rl',
                     'rollingJoint_rr', 'rollingJoint_fr']

# Wheel geometry, measured in the scene (positions in the trunk_respondable frame).
# Re-measure these values if the base model changes.
#   WHEEL_RADIUS (r):    the wheel_respondable_* shapes are pure spheres with
#                        0.1 m diameter, so r = 0.05 m.
#   HALF_WHEELBASE (lx): |x| of the rollingJoint_* positions = 0.228 m.
#   HALF_TRACK (ly):     |y| of the rollingJoint_* positions = 0.1585 m
#                        (left wheels at y = 0.15856, right wheels at y = -0.15848).
WHEEL_RADIUS = 0.05
HALF_WHEELBASE = 0.228
HALF_TRACK = 0.1585

# Compensation factors of the mecanum inverse kinematics. They convert the base
# twist (vx, vy in m/s, wz in rad/s, all in the base frame) into wheel joint
# velocities (rad/s):
#   omega_i = LINEAR_GAIN * (+/-vx +/- vy) +/- ANGULAR_GAIN * wz
#
#   LINEAR_GAIN  = 1/r = 1/0.05 = 20
#     A wheel of radius r spinning at omega rad/s moves its centre at
#     v = omega * r, so omega = v / r.
#   ANGULAR_GAIN = (lx + ly)/r = (0.228 + 0.1585)/0.05 = 7.73
#     With 45-degree rollers, a base rotation wz moves each wheel along its
#     rolling direction at (lx + ly) * wz, so omega = (lx + ly) * wz / r.
#
# The rollingJoint_* z-axes point along -y of the base, so a positive joint
# velocity drives the base backwards. That is why vx enters with a minus sign
# in wheel_velocities().
#
# Verified in CoppeliaSim 4.7.0 rev4: a command of vx = 0.1 m/s moves the base
# at 0.098 m/s, and wz = 0.2 rad/s rotates it at 0.199 rad/s.
LINEAR_GAIN = 1.0 / WHEEL_RADIUS
ANGULAR_GAIN = (HALF_WHEELBASE + HALF_TRACK) / WHEEL_RADIUS

# Stop the base if no command arrives for this long (wall-clock seconds).
# Set to None to hold the last command indefinitely (Lua script behaviour).
CMD_TIMEOUT_SEC = 0.5


class TwistSubscriber(Node):
    """Keeps the latest (vx, vy, wz) received on a TwistStamped topic."""

    def __init__(self, node_name: str, topic: str):
        super().__init__(node_name)
        self._vx = self._vy = self._wz = 0.0
        self._last_msg_time = None
        # Best-effort subscribers match both reliable and best-effort publishers.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(TwistStamped, topic, self._callback, qos)

    def _callback(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._vy = msg.twist.linear.y
        self._wz = msg.twist.angular.z
        self._last_msg_time = time.monotonic()

    def command(self):
        if self._last_msg_time is None:
            return 0.0, 0.0, 0.0
        if (CMD_TIMEOUT_SEC is not None and
                time.monotonic() - self._last_msg_time > CMD_TIMEOUT_SEC):
            return 0.0, 0.0, 0.0
        return self._vx, self._vy, self._wz


def find_wheel_joints(sim, base_handle, names):
    """Finds joints by alias anywhere in the base object's tree."""
    by_alias = {}
    for h in sim.getObjectsInTree(base_handle, sim.object_joint_type, 0):
        by_alias.setdefault(sim.getObjectAlias(h), h)
    missing = [n for n in names if n not in by_alias]
    if missing:
        raise RuntimeError(f"wheel joints not found: {missing}. "
                           f"Joints in tree: {sorted(by_alias)}")
    return [by_alias[n] for n in names]


def wheel_velocities(vx, vy, wz):
    """Mecanum inverse kinematics, matching the Lua script (fl, rl, rr, fr)."""
    fb = LINEAR_GAIN * vx
    lr = LINEAR_GAIN * vy
    rot = ANGULAR_GAIN * wz
    return [-fb + lr + rot,
            -fb - lr + rot,
            -fb + lr - rot,
            -fb - lr - rot]


def measured_twist(sim, base_handle):
    """Linear and angular velocities of the base, expressed in the base frame."""
    lin, ang = sim.getObjectVelocity(base_handle)
    m = sim.getObjectMatrix(base_handle, sim.handle_world)
    # Rotation from base to world: R[r][c] = m[4*r + c]. World to base is R^T.
    to_base = lambda v: [sum(m[4 * r + c] * v[r] for r in range(3))
                         for c in range(3)]
    return to_base(lin), to_base(ang)


def sysCall_init():
    sim = require('sim')
    self.node = None
    self.ros_initialized = False
    self.wheel_joints = []
    try:
        self.base = sim.getObject(ROBOT_BASE_PATH)
        self.wheel_joints = find_wheel_joints(sim, self.base, WHEEL_JOINT_NAMES)

        if not rclpy.ok():
            rclpy.init()
            self.ros_initialized = True
        self.node = TwistSubscriber(NODE_NAME, TWIST_TOPIC)
        self.measured_twist_pub = self.node.create_publisher(
            TwistStamped, MEASURED_TWIST_TOPIC, 10)

        print(f"Holonomic base listening on '{TWIST_TOPIC}' and publishing "
              f"on '{MEASURED_TWIST_TOPIC}' "
              f"(base='{sim.getObjectAlias(self.base, 1)}')")
    except Exception as e:
        print(f"Error initializing holonomic base script: {e}")


def sysCall_actuation():
    if self.node is None:
        return
    sim = require('sim')
    rclpy.spin_once(self.node, timeout_sec=0.0)
    vx, vy, wz = self.node.command()
    for handle, vel in zip(self.wheel_joints, wheel_velocities(vx, vy, wz)):
        sim.setJointTargetVelocity(handle, vel)


def sysCall_sensing():
    if self.node is None:
        return
    sim = require('sim')
    lin, ang = measured_twist(sim, self.base)
    msg = TwistStamped()
    msg.header.stamp = self.node.get_clock().now().to_msg()
    msg.header.frame_id = sim.getObjectAlias(self.base)
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = lin
    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ang
    self.measured_twist_pub.publish(msg)


def sysCall_cleanup():
    if self.node is not None:
        self.node.destroy_node()
        self.node = None
    if self.ros_initialized:
        rclpy.shutdown()
        self.ros_initialized = False
    print("ROS 2 cleaned up")
