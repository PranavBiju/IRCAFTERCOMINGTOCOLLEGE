#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        # Subscribe to user input
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishers
        self.pivot_pub = self.create_publisher(
            Float64MultiArray, '/pivot_controller/commands', 10)
        self.drive_pub = self.create_publisher(
            Float64MultiArray, '/drive_controller/commands', 10)

        # Robot dimensions (meters)
        self.L = 0.6
        self.W = 0.4

        # Small threshold for "zero"
        self.eps = 1e-3

        self.get_logger().info("Improved Swerve Controller Started")

    # ---------------- Utility Functions ----------------

    def wrap_to_pi(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def optimize(self, speed, angle):
        """
        Classic swerve optimization:
        Minimize steering rotation by possibly flipping wheel direction.
        """
        angle = self.wrap_to_pi(angle)

        if abs(angle) > math.pi / 2:
            angle = self.wrap_to_pi(angle + math.pi)
            speed = -speed

        return speed, angle

    # ---------------- Main Callback ----------------

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z

        # Epsilon-based stop
        if (abs(vx) < self.eps and
            abs(vy) < self.eps and
            abs(w)  < self.eps):
            self.stop_robot()
            return

        half_L = self.L / 2.0
        half_W = self.W / 2.0

        # --- Wheel velocity components ---

        # Front Left
        v_fl_x = vx - w * half_W
        v_fl_y = vy + w * half_L

        # Front Right
        v_fr_x = vx + w * half_W
        v_fr_y = vy + w * half_L

        # Back Left
        v_bl_x = vx - w * half_W
        v_bl_y = vy - w * half_L

        # Back Right
        v_br_x = vx + w * half_W
        v_br_y = vy - w * half_L

        # --- Convert to speed & angle ---

        speed_fl = math.hypot(v_fl_x, v_fl_y)
        angle_fl = math.atan2(v_fl_y, v_fl_x)

        speed_fr = math.hypot(v_fr_x, v_fr_y)
        angle_fr = math.atan2(v_fr_y, v_fr_x)

        speed_bl = math.hypot(v_bl_x, v_bl_y)
        angle_bl = math.atan2(v_bl_y, v_bl_x)

        speed_br = math.hypot(v_br_x, v_br_y)
        angle_br = math.atan2(v_br_y, v_br_x)

        # --- Classic swerve optimization ---

        speed_fl, angle_fl = self.optimize(speed_fl, angle_fl)
        speed_fr, angle_fr = self.optimize(speed_fr, angle_fr)
        speed_bl, angle_bl = self.optimize(speed_bl, angle_bl)
        speed_br, angle_br = self.optimize(speed_br, angle_br)

        # --- Publish (alphabetical order: BL, BR, FL, FR) ---

        pivot_msg = Float64MultiArray()
        pivot_msg.data = [angle_bl, angle_br, angle_fl, angle_fr]
        self.pivot_pub.publish(pivot_msg)

        drive_msg = Float64MultiArray()
        drive_msg.data = [speed_bl, speed_br, speed_fl, speed_fr]
        self.drive_pub.publish(drive_msg)

    # ---------------- Stop ----------------

    def stop_robot(self):
        drive_msg = Float64MultiArray()
        drive_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.drive_pub.publish(drive_msg)
        # Intentionally NOT touching pivot angles

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
