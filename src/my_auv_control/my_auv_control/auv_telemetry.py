#!/usr/bin/env python3
"""AUV Telemetry | QoS-aware + Debug"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math, sys

class AUVTelemetry(Node):
    def __init__(self):
        super().__init__('auv_telemetry')
        # QoS для совместимости с Gazebo Harmonic (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, qos)
        self.create_subscription(Imu, '/model/submarine/imu', self.imu_cb, qos)
        
        self.vx=0.0; self.vy=0.0; self.vz=0.0
        self.wx=0.0; self.wy=0.0; self.wz=0.0
        self.roll=0.0; self.pitch=0.0; self.yaw=0.0
        self.got_odom = False
        self.timer = self.create_timer(0.1, self.print_loop)

    def odom_cb(self, msg):
        if not self.got_odom:
            self.get_logger().info("✅ Odometry connected! Receiving data...")
        self.got_odom = True
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def imu_cb(self, msg):
        self.wx = msg.angular_velocity.x
        self.wy = msg.angular_velocity.y
        self.wz = msg.angular_velocity.z

    def print_loop(self):
        if not self.got_odom:
            sys.stdout.write("\r⏳ Waiting for odometry... (checking data flow)")
            sys.stdout.flush()
            return
        out = f"\r[TEL] V:[{self.vx:+3.2f} {self.vy:+3.2f} {self.vz:+3.2f}] m/s | W:[{self.wx:+3.2f} {self.wy:+3.2f} {self.wz:+3.2f}] rad/s | Yaw:{math.degrees(self.yaw):+6.1f}° Pit:{math.degrees(self.pitch):+5.1f}° Roll:{math.degrees(self.roll):+5.1f}°"
        sys.stdout.write(out)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = AUVTelemetry()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
