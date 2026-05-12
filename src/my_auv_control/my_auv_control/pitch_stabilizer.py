#!/usr/bin/env python3
# ============================================================================
# Стабилизация тангажа + подавление перекрёстных связей (yaw-pitch)
# + независимое управление 4 балластными баками (сила, шаг 1 Н)
# ============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import subprocess
import sys
import termios
import tty
import select
import signal
import atexit
import math

# ----------------------------------------------------------------------------
# 1. ВХОДНЫЕ ДАННЫЕ
# ----------------------------------------------------------------------------
WORLD = "static_world"
MODEL = "submarine"

# Параметры стабилизации (подавление перекрёстных связей)
KP_YAW = 8.0
KP_PITCH = 8.0
KD = 2.0
KI = 0.1

# Параметры балластных баков (сила, Н)
STEP_FORCE = 1.0          # шаг изменения силы на бак
MIN_FORCE = -100.0
MAX_FORCE = 100.0

# ----------------------------------------------------------------------------
# 2. ФУНКЦИИ ВЗАИМОДЕЙСТВИЯ С GAZEBO
# ----------------------------------------------------------------------------
def send_force(tank, force):
    if abs(force) < 0.1:
        return
    cmd = [
        "gz", "topic", "-t", f"/world/{WORLD}/wrench/persistent",
        "-m", "gz.msgs.EntityWrench",
        "-p", f'entity: {{name: "{MODEL}::ballast_{tank}", type: LINK}}, reference_frame: "world", wrench: {{force: {{z: {force}}}}}'
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ----------------------------------------------------------------------------
# 3. ОСНОВНОЙ КЛАСС
# ----------------------------------------------------------------------------
class PitchStabilizer(Node):
    def __init__(self):
        super().__init__('pitch_stabilizer')

        # Состояния баков
        self.forces = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}

        # Данные IMU
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.target_yaw = 0.0
        self.target_pitch = 0.0

        # ПИД‑переменные
        self.integral_yaw = 0.0
        self.prev_error_yaw = 0.0
        self.integral_pitch = 0.0
        self.prev_error_pitch = 0.0

        # Публикаторы рулей
        self.pub_vert = self.create_publisher(Float64, "/model/submarine/joint/vertical_rudder/cmd_position", 10)
        self.pub_horiz_left = self.create_publisher(Float64, "/model/submarine/joint/horizontal_rudder_left/cmd_position", 10)
        self.pub_horiz_right = self.create_publisher(Float64, "/model/submarine/joint/horizontal_rudder_right/cmd_position", 10)

        # Подписка на IMU
        self.imu_sub = self.create_subscription(Imu, f"/model/{MODEL}/imu", self.imu_cb, 10)

        # Таймер стабилизации (20 Гц)
        self.timer = self.create_timer(0.05, self.stabilize)

        # Настройка терминала для чтения клавиш без Enter
        self._setup_terminal()

        self.get_logger().info("PitchStabilizer: стабилизация + подавление перекрёстных связей")
        self.get_logger().info("Балласт: u/j – FL, i/k – FR, o/l – RL, p/; – RR, 0 – reset, q – quit")

    def _setup_terminal(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        atexit.register(self._restore)
        signal.signal(signal.SIGINT, self._handler)
        signal.signal(signal.SIGTERM, self._handler)
        tty.setraw(self.fd)

    def _restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def _handler(self, signum, frame):
        self._restore()
        sys.exit(0)

    def imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.asin(max(-1.0, min(1.0, sinp)))

    def stabilize(self):
        dt = 0.05
        # Ошибки (целевые углы пока нулевые – можно привязать к клавишам)
        err_yaw = self.target_yaw - self.current_yaw
        err_pitch = self.target_pitch - self.current_pitch

        # Интегралы
        self.integral_yaw += err_yaw * dt
        self.integral_pitch += err_pitch * dt
        # Анти-виндап
        self.integral_yaw = max(-10.0, min(10.0, self.integral_yaw))
        self.integral_pitch = max(-10.0, min(10.0, self.integral_pitch))

        # Дифференциалы
        d_yaw = (err_yaw - self.prev_error_yaw) / dt
        d_pitch = (err_pitch - self.prev_error_pitch) / dt
        self.prev_error_yaw = err_yaw
        self.prev_error_pitch = err_pitch

        # ПИД + перекрёстное подавление
        control_yaw = (KP_YAW * err_yaw + KI * self.integral_yaw + KD * d_yaw) - 0.5 * err_pitch
        control_pitch = (KP_PITCH * err_pitch + KI * self.integral_pitch + KD * d_pitch) - 0.5 * err_yaw

        max_angle = 0.785
        control_yaw = max(-max_angle, min(max_angle, control_yaw))
        control_pitch = max(-max_angle, min(max_angle, control_pitch))

        self.pub_vert.publish(Float64(data=control_yaw))
        self.pub_horiz_left.publish(Float64(data=control_pitch))
        self.pub_horiz_right.publish(Float64(data=control_pitch))

    def adjust_force(self, tank, delta):
        new = self.forces[tank] + delta
        new = max(MIN_FORCE, min(MAX_FORCE, new))
        if new != self.forces[tank]:
            self.forces[tank] = new
            send_force(tank, new)

    def reset(self):
        for t in self.forces:
            self.forces[t] = 0.0
            send_force(t, 0.0)

    def display(self):
        sys.stdout.write(
            f"\rFL:{self.forces['fl']:+4.0f} FR:{self.forces['fr']:+4.0f} "
            f"RL:{self.forces['rl']:+4.0f} RR:{self.forces['rr']:+4.0f}    "
        )
        sys.stdout.flush()

    def run(self):
        self.display()
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch = sys.stdin.read(1)
                if ch == 'u': self.adjust_force('fl',  STEP_FORCE)
                elif ch == 'j': self.adjust_force('fl', -STEP_FORCE)
                elif ch == 'i': self.adjust_force('fr',  STEP_FORCE)
                elif ch == 'k': self.adjust_force('fr', -STEP_FORCE)
                elif ch == 'o': self.adjust_force('rl',  STEP_FORCE)
                elif ch == 'l': self.adjust_force('rl', -STEP_FORCE)
                elif ch == 'p': self.adjust_force('rr',  STEP_FORCE)
                elif ch == ';': self.adjust_force('rr', -STEP_FORCE)
                elif ch == '0': self.reset()
                elif ch == 'q': break
                self.display()
            rclpy.spin_once(self, timeout_sec=0.0)

# ----------------------------------------------------------------------------
# 4. ГЛАВНАЯ ФУНКЦИЯ
# ----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PitchStabilizer()
    try:
        node.run()
    finally:
        node._restore()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
