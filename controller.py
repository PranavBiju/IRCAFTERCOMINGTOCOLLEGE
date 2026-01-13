#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class CmdVelTeleop(Node):
    def __init__(self):
        super().__init__('cmd_vel_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # velocities
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # increments
        self.lin_step = 0.1
        self.ang_step = 0.2

        self.get_logger().info("""
CMD_VEL TELEOP (for swerve)

w / s : forward / backward
a / d : strafe left / right
q / e : rotate left / right
space : stop
x     : zero everything
CTRL+C: quit
        """)

    def run(self):
        while rclpy.ok():
            key = get_key()

            if key == 'w':
                self.vx += self.lin_step
            elif key == 's':
                self.vx -= self.lin_step
            elif key == 'a':
                self.vy += self.lin_step
            elif key == 'd':
                self.vy -= self.lin_step
            elif key == 'q':
                self.wz += self.ang_step
            elif key == 'e':
                self.wz -= self.ang_step
            elif key == ' ' or key == 'x':
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
            else:
                continue

            self.publish()

    def publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz

        self.pub.publish(msg)

        self.get_logger().info(
            f"cmd_vel | vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}"
        )


def main():
    rclpy.init()
    node = CmdVelTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
