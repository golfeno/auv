#!/usr/bin/env python3
# ============================================================================
# Модуль операторского управления AUV с клавиатуры
# ============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select
import signal
import atexit

# ============================================================================
# 1. ВХОДНЫЕ ДАННЫЕ (конфигурация)
# ============================================================================
# --- Параметры управления ---
FORCE_STEP = 2.0         # шаг изменения тяги (Н) на один винт
MAX_FORCE = 30.0         # максимальная тяга на винт (Н)
RUDDER_STEP = 0.05       # шаг изменения угла рулей (рад)
MAX_RUDDER = 0.785       # максимальный угол рулей (45°)

# --- Топики Gazebo ---
TOPIC_LEFT_FORCE   = "/model/submarine/joint/left_propeller_joint/cmd_force"
TOPIC_RIGHT_FORCE  = "/model/submarine/joint/right_propeller_joint/cmd_force"
TOPIC_VERT_RUDDER  = "/model/submarine/joint/vertical_rudder/cmd_position"
TOPIC_HORIZ_LEFT   = "/model/submarine/joint/horizontal_rudder_left/cmd_position"
TOPIC_HORIZ_RIGHT  = "/model/submarine/joint/horizontal_rudder_right/cmd_position"

# --- Маппинг клавиш (команды) ---
KEY_COMMAND = {
    'z': 'gas', 'x': 'brake',
    'a': 'turn_left', 'd': 'turn_right',
    'f': 'rudder_left', 'g': 'rudder_right',
    'w': 'pitch_up', 's': 'pitch_down',
    'q': 'roll_left', 'e': 'roll_right',
    ' ': 'stop',
    'q': 'quit'
}

# ============================================================================
# 2. ФУНКЦИИ ПРЕОБРАЗОВАНИЯ (ограничение значений)
# ============================================================================
def limit_force(force):
    return max(-MAX_FORCE, min(MAX_FORCE, force))

def limit_angle(angle):
    return max(-MAX_RUDDER, min(MAX_RUDDER, angle))

# ============================================================================
# 3. ФУНКЦИИ ВЗАИМОДЕЙСТВИЯ С GAZEBO (публикация команд)
# ============================================================================
class AUVController(Node):
    def __init__(self):
        super().__init__('auv_controller')
        # Создаём публикаторы
        self.pub_left = self.create_publisher(Float64, TOPIC_LEFT_FORCE, 10)
        self.pub_right = self.create_publisher(Float64, TOPIC_RIGHT_FORCE, 10)
        self.pub_vert = self.create_publisher(Float64, TOPIC_VERT_RUDDER, 10)
        self.pub_horiz_left = self.create_publisher(Float64, TOPIC_HORIZ_LEFT, 10)
        self.pub_horiz_right = self.create_publisher(Float64, TOPIC_HORIZ_RIGHT, 10)

        # Состояния (текущие значения)
        self.left_force = 0.0
        self.right_force = 0.0
        self.vert_angle = 0.0
        self.horiz_left = 0.0
        self.horiz_right = 0.0

        # Настройка терминала для чтения клавиш без Enter
        self._setup_terminal()

        self.get_logger().info("=== AUV Keyboard Control ===")
        self.get_logger().info("  Z/X  - gas/brake (thrust)")
        self.get_logger().info("  A/D  - turn in place (differential thrust)")
        self.get_logger().info("  F/G  - rudder (yaw)")
        self.get_logger().info("  W/S  - pitch (elevator)")
        self.get_logger().info("  Q/E  - roll (aileron)")
        self.get_logger().info("  Space - emergency stop")
        self.get_logger().info("  Q     - quit")

    def _setup_terminal(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        atexit.register(self._restore_terminal)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        tty.setraw(self.fd)

    def _restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def _signal_handler(self, signum, frame):
        self._restore_terminal()
        sys.exit(0)

    # ------------------------------------------------------------------------
    # Команды управления (изменение состояний)
    # ------------------------------------------------------------------------
    def gas(self):
        self.left_force += FORCE_STEP
        self.right_force = self.left_force
        self._clamp_forces()

    def brake(self):
        self.left_force -= FORCE_STEP
        self.right_force = self.left_force
        self._clamp_forces()

    def turn_left(self):
        self.left_force += FORCE_STEP
        self.right_force -= FORCE_STEP
        self._clamp_forces()

    def turn_right(self):
        self.left_force -= FORCE_STEP
        self.right_force += FORCE_STEP
        self._clamp_forces()

    def rudder_left(self):
        self.vert_angle -= RUDDER_STEP
        self.vert_angle = limit_angle(self.vert_angle)

    def rudder_right(self):
        self.vert_angle += RUDDER_STEP
        self.vert_angle = limit_angle(self.vert_angle)

    def pitch_up(self):
        self.horiz_left += RUDDER_STEP
        self.horiz_right += RUDDER_STEP
        self._clamp_horiz()

    def pitch_down(self):
        self.horiz_left -= RUDDER_STEP
        self.horiz_right -= RUDDER_STEP
        self._clamp_horiz()

    def roll_left(self):
        self.horiz_left += RUDDER_STEP
        self.horiz_right -= RUDDER_STEP
        self._clamp_horiz()

    def roll_right(self):
        self.horiz_left -= RUDDER_STEP
        self.horiz_right += RUDDER_STEP
        self._clamp_horiz()

    def stop(self):
        self.left_force = 0.0
        self.right_force = 0.0
        self.vert_angle = 0.0
        self.horiz_left = 0.0
        self.horiz_right = 0.0

    def _clamp_forces(self):
        self.left_force = limit_force(self.left_force)
        self.right_force = limit_force(self.right_force)

    def _clamp_horiz(self):
        self.horiz_left = limit_angle(self.horiz_left)
        self.horiz_right = limit_angle(self.horiz_right)

    # ------------------------------------------------------------------------
    # 4. ФУНКЦИИ ВЫВОДА (отображение состояния)
    # ------------------------------------------------------------------------
    def _publish_all(self):
        self.pub_left.publish(Float64(data=self.left_force))
        self.pub_right.publish(Float64(data=self.right_force))
        self.pub_vert.publish(Float64(data=self.vert_angle))
        self.pub_horiz_left.publish(Float64(data=self.horiz_left))
        self.pub_horiz_right.publish(Float64(data=self.horiz_right))

    def _display_state(self):
        # Вывод в одну строку
        sys.stdout.write(
            f"\rL:{self.left_force:+5.1f} R:{self.right_force:+5.1f} "
            f"V:{self.vert_angle:+5.2f} HL:{self.horiz_left:+5.2f} HR:{self.horiz_right:+5.2f}    "
        )
        sys.stdout.flush()

    # ------------------------------------------------------------------------
    # 5. ОСНОВНОЙ ЦИКЛ (чтение клавиш)
    # ------------------------------------------------------------------------
    def run(self):
        # Сначала публикуем начальное состояние (нули)
        self._publish_all()
        self._display_state()

        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch = sys.stdin.read(1)
                # Обработка команд
                if ch == 'z': self.gas()
                elif ch == 'x': self.brake()
                elif ch == 'a': self.turn_left()
                elif ch == 'd': self.turn_right()
                elif ch == 'f': self.rudder_left()
                elif ch == 'g': self.rudder_right()
                elif ch == 'w': self.pitch_up()
                elif ch == 's': self.pitch_down()
                elif ch == 'q': self.roll_left()
                elif ch == 'e': self.roll_right()
                elif ch == ' ': self.stop()
                elif ch == 'q' or ch == '\x03':  # q или Ctrl+C
                    break
                else:
                    continue
                # После каждого действия отправляем команды и обновляем экран
                self._publish_all()
                self._display_state()
            # Небольшая пауза для ROS
            rclpy.spin_once(self, timeout_sec=0.0)


# ============================================================================
# ГЛАВНАЯ ФУНКЦИЯ
# ============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    try:
        node.run()
    finally:
        node._restore_terminal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
