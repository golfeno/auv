import sys
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .models import VehicleState, PHASE_TRANSLATION

class TelemetryManager:
    def __init__(self, node: Node):
        self.node = node
        self.pub_p_nav = node.create_publisher(Point, '/analytics/phase_nav', 10)
        self.pub_p_stab = node.create_publisher(Point, '/analytics/phase_stab', 10)
        self.pub_status = node.create_publisher(String, '/model/submarine/status', 10)
        self.last_log = 0.0

    def publish_analytics(self, state: VehicleState, phase: str):
        p = Point(x=state.pos[0], y=state.pos[1], z=state.pos[2])
        if phase in ['NAV', 'XY_FINAL']: self.pub_p_nav.publish(p)
        elif phase in ['Z_STAB', 'HOVER_STAB']: self.pub_p_stab.publish(p)
        msg = String(); msg.data = f"WP_{phase.split('_')[0]}_{phase}"
        self.pub_status.publish(msg)

    def log_console(self, state: VehicleState, phase: str, wp_idx: int, clock_now: float):
        if clock_now - self.last_log < 0.05: return
        self.last_log = clock_now
        sys.stdout.write(f"\r[v49.10] Тчк:{wp_idx+1} | Фаза: {PHASE_TRANSLATION.get(phase, phase)} | "
                         f"XYZ: ({state.pos[0]:.1f}, {state.pos[1]:.1f}, {state.pos[2]:.1f}) | "
                         f"V: {state.vel:+.2f}м/с | D2D: {state.dist_2d:.2f}м | D3D: {state.dist_3d:.2f}м | Z_Err: {state.z_err:+.2f}м")
        sys.stdout.flush()
