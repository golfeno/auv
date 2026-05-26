from typing import List
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .models import VehicleState, ControlConfig

class SensorFusion:
    def __init__(self, node: Node):
        self.node = node
        self.state = VehicleState()
        self.prev_baro_z = 0.0
        self.prev_rpy = [0.0, 0.0, 0.0]
        self.node.create_subscription(Odometry, '/model/submarine/odometry', self._odom_cb, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(Float32, '/model/submarine/pressure', self._press_cb, qos)

    def _press_cb(self, msg: Float32):
        self.state.baro_z = (ControlConfig.P_Z0 - msg.data) / ControlConfig.RHO_G

    def _odom_cb(self, msg: Odometry):
        self.state.pos[0] = msg.pose.pose.position.x
        self.state.pos[1] = msg.pose.pose.position.y
        self.state.pos[2] = self.state.baro_z
        self.state.vel = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.state.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.state.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.state.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def update(self, target: List[float], dt: float):
        s = self.state
        dx, dy, dz = target[0]-s.pos[0], target[1]-s.pos[1], target[2]-s.pos[2]
        s.dist_2d = math.hypot(dx, dy)
        s.dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        s.bearing = math.atan2(dy, dx)
        s.z_err = s.pos[2] - target[2]
        s.roll_abs = abs(s.rpy[0]); s.pitch_curr = s.rpy[1]
        raw_dz = (s.pos[2] - self.prev_baro_z) / dt
        s.dz_dt = 0.6 * s.dz_dt + 0.4 * raw_dz
        self.prev_baro_z = s.pos[2]
        s.yaw_err = math.atan2(math.sin(s.bearing - s.rpy[2]), math.cos(s.bearing - s.rpy[2]))
        s.pitch_d = (s.rpy[1] - self.prev_rpy[1]) / dt
        s.yaw_d = (s.rpy[2] - self.prev_rpy[2]) / dt
        self.prev_rpy = list(s.rpy)
