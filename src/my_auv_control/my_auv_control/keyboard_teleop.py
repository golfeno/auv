#!/usr/bin/env python3
"""
ROS 2 Keyboard Teleop for AUV (Gazebo Harmonic Compatible)
Использует rclpy.Timer, параметры, чистый выход и готов к интеграции ПИД.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

class AUVKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('auv_keyboard_teleop')
        self.declare_parameter('force_step', 2.0)
        self.declare_parameter('max_force', 30.0)
        self.declare_parameter('rudder_step', 0.05)
        self.declare_parameter('max_rudder', 0.785)
        self.declare_parameter('publish_rate', 20.0)

        self.force_step = self.get_parameter('force_step').value
        self.max_force = self.get_parameter('max_force').value
        self.rudder_step = self.get_parameter('rudder_step').value
        self.max_rudder = self.get_parameter('max_rudder').value
        rate = self.get_parameter('publish_rate').value

        self.pub_left = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_right = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_horiz_l = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_horiz_r = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)

        self.state = {
            'left_force': 0.0, 'right_force': 0.0,
            'vert_rudder': 0.0,
            'horiz_left': 0.0, 'horiz_right': 0.0
        }

        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info("🎮 AUV Keyboard Teleop Initialized")
        self._print_help()

    def _limit(self, value, limit):
        return max(-limit, min(limit, value))

    def _print_help(self):
        self.get_logger().info(
            "Controls:\n"
            "  W / S   → Pitch (горизонтальные рули)\n"
            "  A / D   → Yaw (вертикальный руль)\n"
            "  I / K   → Thrust (оба винта)\n"
            "  J / L   → Differential Turn (разворот на месте)\n"
            "  SPACE   → Emergency Stop (сброс в 0)\n"
            "  ESC / Q → Quit"
        )

    def _read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def timer_callback(self):
        key = self._read_key()
        if not key:
            self._publish_state()
            return

        if key == 'w':
            self.state['horiz_left'] = self._limit(self.state['horiz_left'] + self.rudder_step, self.max_rudder)
            self.state['horiz_right'] = self._limit(self.state['horiz_right'] + self.rudder_step, self.max_rudder)
        elif key == 's':
            self.state['horiz_left'] = self._limit(self.state['horiz_left'] - self.rudder_step, self.max_rudder)
            self.state['horiz_right'] = self._limit(self.state['horiz_right'] - self.rudder_step, self.max_rudder)
        elif key == 'a':
            self.state['vert_rudder'] = self._limit(self.state['vert_rudder'] + self.rudder_step, self.max_rudder)
        elif key == 'd':
            self.state['vert_rudder'] = self._limit(self.state['vert_rudder'] - self.rudder_step, self.max_rudder)
        elif key == 'i':
            self.state['left_force'] = self._limit(self.state['left_force'] + self.force_step, self.max_force)
            self.state['right_force'] = self._limit(self.state['right_force'] + self.force_step, self.max_force)
        elif key == 'k':
            self.state['left_force'] = self._limit(self.state['left_force'] - self.force_step, self.max_force)
            self.state['right_force'] = self._limit(self.state['right_force'] - self.force_step, self.max_force)
        elif key == 'j':
            self.state['left_force'] = self._limit(self.state['left_force'] + self.force_step, self.max_force)
            self.state['right_force'] = self._limit(self.state['right_force'] - self.force_step, self.max_force)
        elif key == 'l':
            self.state['left_force'] = self._limit(self.state['left_force'] - self.force_step, self.max_force)
            self.state['right_force'] = self._limit(self.state['right_force'] + self.force_step, self.max_force)
        elif key == ' ':
            for k in self.state: self.state[k] = 0.0
            self.get_logger().warn("🛑 EMERGENCY STOP")
        elif key in ['\x1b', 'q']:
            self.get_logger().info("🚪 Quit requested.")
            self.destroy_timer(self.timer)
            return

        self._publish_state()

    def _publish_state(self):
        self.pub_left.publish(Float64(data=self.state['left_force']))
        self.pub_right.publish(Float64(data=self.state['right_force']))
        self.pub_vert.publish(Float64(data=self.state['vert_rudder']))
        self.pub_horiz_l.publish(Float64(data=self.state['horiz_left']))
        self.pub_horiz_r.publish(Float64(data=self.state['horiz_right']))

    def _cleanup(self):
        for k in self.state: self.state[k] = 0.0
        self._publish_state()
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.get_logger().info("🔌 Node shutdown. Terminal restored.")

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self._cleanup()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AUVKeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
