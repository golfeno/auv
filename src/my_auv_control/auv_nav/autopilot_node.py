import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .models import ActuatorCommands, ControlConfig
from .sensor_fusion import SensorFusion
from .phase_manager import PhaseManager
from .pid_controller import PIDController
from .telemetry import TelemetryManager

class AUVAutopilotNode(Node):
    def __init__(self, waypoints):
        super().__init__('auv_ctrl')
        if not waypoints: raise ValueError("Waypoint list cannot be empty")
        self.phase_mgr = PhaseManager(waypoints)
        self.sensors = SensorFusion(self)
        self.controller = PIDController()
        self.telemetry = TelemetryManager(self)
        self.pub_lt = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        self.timer = self.create_timer(ControlConfig.dt, self._control_loop)
        self.phase_mgr.init_waypoint(self.get_clock().now().nanoseconds / 1e9, [0.0, 0.0, 0.0])

    def _publish_actuators(self, cmd: ActuatorCommands):
        self.pub_lt.publish(Float64(data=cmd.lt))
        self.pub_rt.publish(Float64(data=cmd.rt))
        self.pub_vert.publish(Float64(data=cmd.rv))
        self.pub_hl.publish(Float64(data=cmd.hl))
        self.pub_hr.publish(Float64(data=cmd.hr))

    def _control_loop(self):
        if self.phase_mgr.state == 'FINISH': return
        clock_now = self.get_clock().now().nanoseconds / 1e9
        phase = self.phase_mgr.evaluate(self.sensors.state, clock_now)
        self.sensors.update(self.phase_mgr.target, ControlConfig.dt)
        target_params = self.phase_mgr.get_target_params(self.sensors.state)
        commands = self.controller.compute(self.sensors.state, phase, target_params, ControlConfig.dt)
        self._publish_actuators(commands)
        self.telemetry.publish_analytics(self.sensors.state, phase)
        if phase == 'HOVER_STAB' and abs(self.sensors.state.vel) < 0.18:
            self.get_logger().info(f"Точка {self.phase_mgr.current_wp_idx + 1} достигнута.")
            self.phase_mgr.current_wp_idx += 1
            if self.phase_mgr.current_wp_idx < len(self.phase_mgr.waypoints):
                self.phase_mgr.init_waypoint(clock_now, self.sensors.state.pos)
            else:
                self.phase_mgr.state = 'FINISH'
                self._publish_actuators(ActuatorCommands())
                self.get_logger().info("Миссия успешно завершена!")
                raise SystemExit
        else:
            self.telemetry.log_console(self.sensors.state, phase, self.phase_mgr.current_wp_idx, clock_now)

def main():
    wps = []
    print("Введите waypoints (X Y Z). Пустая строка для завершения.")
    wp_count = 1
    while True:
        try:
            inp = input(f"Точка {wp_count} (X Y Z) или Enter: ").strip()
            if not inp:
                if not wps: continue
                break
            parts = inp.split()
            if len(parts) != 3: continue
            wps.append(tuple(map(float, parts)))
            wp_count += 1
        except ValueError: pass
    print("[v49.10] Запуск OOP-архитектуры...")
    rclpy.init()
    node = AUVAutopilotNode(wps)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): node._publish_actuators(ActuatorCommands())
    finally: node.destroy_node(); rclpy.shutdown()
