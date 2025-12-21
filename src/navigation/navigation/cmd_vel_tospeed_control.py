import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math


class CmdVelToSpeedControl(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_speed_control")

        self.wheel_base = 0.17
        self.max_steer_deg = 35.0
        self.max_linear_speed = 1.5

        # Store latest commands
        self.latest_v = 0.0
        self.latest_w = 0.0

        # Speed (SAFE, filtered)
        self.sub_speed = self.create_subscription(Twist, "cmd_vel_nav_filtered", self.cmd_vel_cb, 10)

        # Steering (TRUE curvature, unfiltered)
        self.sub_steering = self.create_subscription(Twist, "cmd_vel_nav", self.cmd_ang_cb, 10)

        self.pub = self.create_publisher(Float32MultiArray, "SpeedControl", 10)

    def cmd_vel_cb(self, msg: Twist):
        self.latest_v = msg.linear.x
        self.compute_and_publish()

    def cmd_ang_cb(self, msg: Twist):
        self.latest_w = msg.angular.z
        self.compute_and_publish()

    def compute_and_publish(self):
        v = self.latest_v
        w = self.latest_w

        # --- Speed normalization ---
        speed_norm = max(-1.0, min(1.0, v / self.max_linear_speed))

        # --- If stopped: center steering ---
        if abs(v) < 0.01:
            self.publish_cmd(0.0, 90.0)
            return

        # --- Ackermann steering from curvature ---
        if abs(w) > 1e-4:
            R = v / w
            delta = math.atan(self.wheel_base / R)
        else:
            delta = 0.0

        # Reverse steering when backing up
        if v < 0:
            delta = -delta

        # Clamp steering
        delta_deg = math.degrees(-delta)
        delta_deg = max(-self.max_steer_deg,
                        min(self.max_steer_deg, delta_deg))

        # --- Map to servo ---
        servo_angle = 90.0 + (delta_deg / self.max_steer_deg) * 90.0
        servo_angle = max(0.0, min(180.0, servo_angle))

        self.publish_cmd(speed_norm, servo_angle)

    def publish_cmd(self, speed, servo):
        msg = Float32MultiArray()
        msg.data = [speed, servo]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSpeedControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
