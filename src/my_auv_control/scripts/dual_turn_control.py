#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios

class DualTurnControl(Node):
    def __init__(self):
        super().__init__('dual_turn_control')
        # Публикаторы
        self.pub_left = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_right = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_horiz_left = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_horiz_right = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)

        # Состояния
        self.left_force = 0.0
        self.right_force = 0.0
        self.vert_angle = 0.0      # угол вертикального руля (для поворота в движении)
        self.horiz_left = 0.0
        self.horiz_right = 0.0

        # Шаги и ограничения
        self.force_step = 50.0
        self.angle_step = 0.2
        self.max_force = 500.0
        self.max_angle = 0.785      # ±45°

        self.get_logger().info("Управление:")
        self.get_logger().info("Z/X - газ/тормоз (увеличение/уменьшение тяги)")
        self.get_logger().info("A/D - поворот на месте (разность тяг, руль в нейтрали)")
        self.get_logger().info("F/G - поворот в движении (отклонение вертикального руля)")
        self.get_logger().info("W/S - тангаж (синхронно оба горизонтальных руля)")
        self.get_logger().info("Q/E - крен (бочка, противофаза горизонтальных рулей)")
        self.get_logger().info("Пробел - стоп (обнуление всего)")

    def publish_all(self):
        self.pub_left.publish(Float64(data=self.left_force))
        self.pub_right.publish(Float64(data=self.right_force))
        self.pub_vert.publish(Float64(data=self.vert_angle))
        self.pub_horiz_left.publish(Float64(data=self.horiz_left))
        self.pub_horiz_right.publish(Float64(data=self.horiz_right))
        # Вывод состояния в одну строку
        print(f"\rL:{self.left_force:+5.0f} R:{self.right_force:+5.0f} | V:{self.vert_angle:+.2f} | HL:{self.horiz_left:+.2f} HR:{self.horiz_right:+.2f}   ", end='')

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            # ---- Газ / тормоз (общая тяга) ----
            if key == 'z':
                self.left_force = min(self.max_force, self.left_force + self.force_step)
                self.right_force = self.left_force
            elif key == 'x':
                self.left_force = max(-self.max_force, self.left_force - self.force_step)
                self.right_force = self.left_force

            # ---- Поворот на месте (дифференциал тяги) ----
            elif key == 'a':
                self.left_force = min(self.max_force, self.left_force + self.force_step)
                self.right_force = max(-self.max_force, self.right_force - self.force_step)
                # Руль в нейтраль при повороте на месте (необязательно)
                # self.vert_angle = 0.0
            elif key == 'd':
                self.left_force = max(-self.max_force, self.left_force - self.force_step)
                self.right_force = min(self.max_force, self.right_force + self.force_step)

            # ---- Поворот в движении (отклонение руля) ----
            elif key == 'f':
                self.vert_angle = min(self.max_angle, self.vert_angle + self.angle_step)
            elif key == 'g':
                self.vert_angle = max(-self.max_angle, self.vert_angle - self.angle_step)

            # ---- Тангаж (синхронно) ----
            elif key == 'w':
                self.horiz_left = min(self.max_angle, self.horiz_left + self.angle_step)
                self.horiz_right = min(self.max_angle, self.horiz_right + self.angle_step)
            elif key == 's':
                self.horiz_left = max(-self.max_angle, self.horiz_left - self.angle_step)
                self.horiz_right = max(-self.max_angle, self.horiz_right - self.angle_step)

            # ---- Крен (бочка) – противофаза ----
            elif key == 'q':
                self.horiz_left = min(self.max_angle, self.horiz_left + self.angle_step)
                self.horiz_right = max(-self.max_angle, self.horiz_right - self.angle_step)
            elif key == 'e':
                self.horiz_left = max(-self.max_angle, self.horiz_left - self.angle_step)
                self.horiz_right = min(self.max_angle, self.horiz_right + self.angle_step)

            # ---- Стоп ----
            elif key == ' ':
                self.left_force = self.right_force = 0.0
                self.vert_angle = 0.0
                self.horiz_left = self.horiz_right = 0.0

            # ---- Выход ----
            elif key == '\x03':
                break

            # Игнорируем другие клавиши
            else:
                continue

            self.publish_all()
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    node = DualTurnControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
