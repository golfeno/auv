#!/usr/bin/env python3
"""AUV PID Autopilot v31.1 | Hydrodynamic Depth | Calibrated | Stable"""
import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0 = 101325.0
RHO_G = 9810.0

# 🔥 Знак глубины: 1.0 = рули вниз для погружения. Если всплывает -> поменяй на -1.0
DEPTH_SIGN = -1.0

class AUVController(Node):
    def __init__(self):
        super().__init__('auv_ctrl')
        self.pub_lt = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        
        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)
        qos_s = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, '/model/submarine/pressure', self.press_cb, qos_s)

        self.state = 'INIT'
        self.pos = [0.0, 0.0, 0.0]; self.vel = 0.0
        self.rpy = [0.0, 0.0, 0.0]; self.prev_rpy = [0.0, 0.0, 0.0]
        self.start_pos = [0.0, 0.0, 0.0]
        self.target_rel = [0.0, 0.0]
        self.bearing = 0.0
        self.dist_to_target = 1000.0
        
        # Инициализация целевых смещений (фикс AttributeError)
        self.rel_x = 0.0; self.rel_y = 0.0; self.rel_z = 0.0
        
        self.depth = 0.0; self.use_pressure = False
        self.target_z = 0.0
        self.prev_depth = 0.0
        
        # 🌀 ПАРАМЕТРЫ
        self.orbit_radius = 5.0
        
        # 🛑 СКОРОСТЬ
        self.max_cruise_speed = 2.0
        self.min_cruise_speed = 0.6
        self.brake_threshold = 0.8
        
        # ПИД ГЛУБИНЫ
        self.Kp_z = 4.0; self.Ki_z = 0.2; self.Kd_z = 1.5
        self.depth_int = 0.0; self.INT_MAX_Z = 0.35
        
        # ПИД КУРСА
        self.Kp_yaw = 0.8; self.Ki_yaw = 0.15; self.Kd_yaw = 0.3
        self.K_diff = 3.0
        self.yaw_int = 0.0; self.INT_MAX_YAW = 1.0
        self.cte = 0.0; self.prev_cte = 0.0
        
        # КРЕН
        self.Kp_roll = 6.0; self.Kd_roll = 2.0
        self.roll_bias = 0.04
        
        self.stable_t = 0.0; self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)

    def press_cb(self, msg):
        self.depth = (P_Z0 - msg.data) / RHO_G
        self.use_pressure = True

    def odom_cb(self, msg):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.vel = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        
        if self.state == 'INIT':
            self.start_pos = [self.pos[0], self.pos[1], self.depth]
            self.target_rel = [self.rel_x, self.rel_y]
            self.target_z = self.depth + self.rel_z
            self.bearing = math.atan2(self.target_rel[1], self.target_rel[0])
            self.prev_rpy = list(self.rpy); self.state = 'STAB'
            self.get_logger().info(f"🎯 Цель: ΔX={self.target_rel[0]:.1f} ΔY={self.target_rel[1]:.1f} Z={self.target_z:+.2f}")

        # Дистанция до цели
        dx_rem = self.target_rel[0] - (self.pos[0] - self.start_pos[0])
        dy_rem = self.target_rel[1] - (self.pos[1] - self.start_pos[1])
        self.dist_to_target = math.hypot(dx_rem, dy_rem)
        
        # CTE
        dx_total = self.target_rel[0]; dy_total = self.target_rel[1]
        L_total = math.hypot(dx_total, dy_total)
        dx_now = self.pos[0] - self.start_pos[0]
        dy_now = self.pos[1] - self.start_pos[1]
        if L_total > 0.1: self.cte = (dy_total * dx_now - dx_total * dy_now) / L_total
        else: self.cte = 0.0

    def loop(self):
        if self.state not in ['STAB', 'NAV']: return
        
        # 🔹 ПИД ГЛУБИНЫ
        depth_err = self.depth - self.target_z
        if abs(depth_err) < 1.0:
            self.depth_int = max(-self.INT_MAX_Z, min(self.INT_MAX_Z, self.depth_int + depth_err * self.dt))
        else: self.depth_int = 0.0
        d_depth = (self.depth - self.prev_depth) / self.dt
        raw_h = DEPTH_SIGN * (self.Kp_z * depth_err + self.Ki_z * self.depth_int + self.Kd_z * d_depth)
        rudder_h = max(-0.45, min(0.45, raw_h))
        self.prev_depth = self.depth

        # 🔹 ПИД КУРСА
        yaw_err = math.atan2(math.sin(self.bearing - self.rpy[2]), math.cos(self.bearing - self.rpy[2]))
        if abs(math.degrees(yaw_err)) < 2.0: yaw_err = 0.0
        cte_correction = 0.3 * self.cte
        total_yaw_err = yaw_err + cte_correction
        
        if abs(total_yaw_err) < 0.005: self.yaw_int = 0.0
        else: self.yaw_int = max(-self.INT_MAX_YAW, min(self.INT_MAX_YAW, self.yaw_int + total_yaw_err * self.dt))
        d_yaw = (self.rpy[2] - self.prev_rpy[2]) / self.dt
        rudder_v = -(self.Kp_yaw * total_yaw_err + self.Ki_yaw * self.yaw_int + self.Kd_yaw * d_yaw)
        rudder_v = max(-0.4, min(0.4, rudder_v))

        # 🔹 КРЕН
        roll_err = self.rpy[0]
        d_roll = (self.rpy[0] - self.prev_rpy[0]) / self.dt
        roll_pid = self.Kp_roll * roll_err + self.Kd_roll * d_roll
        
        cmd_hl = rudder_h - roll_pid - self.roll_bias
        cmd_hr = rudder_h + roll_pid + self.roll_bias
        cmd_hl = max(-0.45, min(0.45, cmd_hl))
        cmd_hr = max(-0.45, min(0.45, cmd_hr))
        self.prev_rpy = list(self.rpy)
        
        thrust = 0.0; cmd_lt = 0.0; cmd_rt = 0.0

        # ================= STATE MACHINE =================
        if self.state == 'STAB':
            if abs(roll_err) < 0.2: self.stable_t += self.dt
            else: self.stable_t = 0.0
            if self.stable_t >= 1.5: 
                self.state = 'NAV'; self.get_logger().info("🟢 NAV | Moving to target...")
            cmd_hl = max(-0.3, min(0.3, -roll_pid - self.roll_bias))
            cmd_hr = max(-0.3, min(0.3, roll_pid + self.roll_bias))

        elif self.state == 'NAV':
            #  ПРОФИЛЬ СКОРОСТИ
            target_speed = max(self.min_cruise_speed, min(self.max_cruise_speed, self.dist_to_target * 0.3))
            
            # 🛑 ТОРМОЖЕНИЕ
            if self.vel > target_speed + self.brake_threshold:
                thrust = 1.5  # Активное торможение
            else:
                thrust = -target_speed * 4.5  # Крейсерская тяга
                
            if abs(math.degrees(total_yaw_err)) > 3.0:
                diff = self.K_diff * total_yaw_err
                cmd_lt = thrust + diff; cmd_rt = thrust - diff
            else: cmd_lt = thrust; cmd_rt = thrust
            
            if self.dist_to_target < 2.5:
                self.state = 'FINISH'
                self.get_logger().info("🏁 FINISH")
                self._pub(0,0,0,0,0)
                print("\n✅ DONE")
                rclpy.shutdown(); return

        self._pub(cmd_lt, cmd_rt, rudder_v, cmd_hl, cmd_hr)
        src = 'P' if self.use_pressure else 'O'
        print(f"\r[{self.state:4}] Pos:[{self.pos[0]:+.1f}, {self.pos[1]:+.1f}, {self.depth:+.2f}] | "
              f"Dist:{self.dist_to_target:.1f}m | V:{self.vel:+.2f} | Z_Err:{depth_err:+.2f} | "
              f"H:{rudder_h:+.2f} V:{rudder_v:+.2f} [{src}]", end='', flush=True)

    def _pub(self, lt, rt, rv, hl, hr):
        self.pub_lt.publish(Float64(data=float(lt))); self.pub_rt.publish(Float64(data=float(rt)))
        self.pub_vert.publish(Float64(data=float(rv))); self.pub_hl.publish(Float64(data=float(hl))); self.pub_hr.publish(Float64(data=float(hr)))

    def run(self):
        try:
            print("="*60 + "\n🚢 AUV v31.1 (Restored | Calibrated)\n" + "="*60)
            try:
                self.rel_x = float(input("📍 ΔX: ")); self.rel_y = float(input("📍 ΔY: ")); self.rel_z = float(input(" ΔZ: "))
            except: self.rel_x=50.0; self.rel_y=0.0; self.rel_z=-5.0
            rclpy.spin(self)
        except KeyboardInterrupt: print("\n🛑 Stop"); self._pub(0,0,0,0,0)

def main():
    rclpy.init(); node = AUVController()
    try: node.run()
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
