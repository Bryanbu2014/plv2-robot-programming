import signal
import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import time


class Tb3(Node):
    def __init__(self):
        super().__init__("tb3")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data,
        )  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.velo = 50
        self.ROBOT_WIDTH = 0.281
        self.tolerance = 0.17  # Ori value 0.17
        self.rotation_buffer = 0.3
        self.scan_values = []
        self.error = self.ROBOT_WIDTH + self.tolerance
        self.state = "go"
        # self.rotation = self.create_timer(0.5, self.rotate_90_degrees_anticlockwise_a)

    def rotate_90_degrees_anticlockwise_a(self):
        """
        Rotate the robot 90 degrees anti-clockwise.
        *Problem: However this is hardcoded, hence it doesn't really work with faster speed.*
        """
        if self.latest_scan_data is not None:
            self.vel(0, 10)

            if (
                0.54 <= self.latest_scan_data.ranges[0] < 0.58
                and 0.35 <= self.latest_scan_data.ranges[180] < 0.40
                and 0.43 <= self.latest_scan_data.ranges[90] < 0.47
                and 0.48 <= self.latest_scan_data.ranges[-90] <= 0.52
            ):
                print("STOP!")
                # self.rotation.cancel()
                self.vel(0, 0)

    def rotate_90_degrees_anticlockwise_b(self):
        """
        Rotate the robot 90 degrees anti-clockwise.
        *Problem: However this is hardcoded, hence it doesn't really work with faster speed.*
        """
        if self.latest_scan_data is not None:
            self.vel(0, 30)

            if self.latest_scan_data.ranges[-90] <= 0.165:
                print("STOP!")
                self.vel(0, 0)
                self.state = "final go"

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """publishes linear and angular self.velocities in percent"""
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """is run whenever a LaserScan msg is received"""
        print()
        print("Distances:")
        print("⬆️ :", msg.ranges[0])
        print("⬇️ :", msg.ranges[180])
        print("⬅️ :", msg.ranges[90])
        print("➡️ :", msg.ranges[-90])
        print(self.state)
        self.latest_scan_data = msg

        if self.state == "go":
            self.go(msg)
        elif self.state == "stop":
            self.vel(0, 0)
            time.sleep(0.5)
            self.state = "rotate 90 degrees anticlockwise"
        elif self.state == "rotate 90 degrees anticlockwise":
            self.rotate_90_degrees_anticlockwise_b()
        elif self.state == "final go":
            self.go(msg)

    def get_avg_distance(self, msg):
        if len(self.scan_values) < 2:
            self.scan_values.append(msg.ranges[0])
            print("Scan values: " + str(self.scan_values))
            avg_distance = self.scan_values[0]
            return avg_distance

        self.scan_values.pop(0)
        self.scan_values.append(msg.ranges[0])
        print("Scan values: " + str(self.scan_values))

        avg_distance = (self.scan_values[0] + self.scan_values[1]) / 2
        return avg_distance

    def go(self, msg):
        avg_distance = self.get_avg_distance(msg)
        print("Average distance: " + str(avg_distance))
        if avg_distance > self.error:
            self.update_velocity("acceleration")
        elif self.state == "go":
            if avg_distance < self.rotation_buffer:
                self.update_velocity("emergency brake")
                self.state = "stop"
        else:
            self.update_velocity("deceleration")

    def update_velocity(self, mode: str):
        """
        Update the velocity of the robot.

        :param mode: To instruct the robot whether it should accelerate or decelerate.
        """
        if mode == "acceleration":
            self.velo = self.velo + 8
            if self.velo >= 100:
                self.velo = 100
        elif mode == "deceleration":
            self.velo = self.velo - 8
            if self.velo <= 0:
                self.velo = 0
        elif mode == "emergency brake":
            self.velo = 0
        self.vel(self.velo)
        print("Velocity:", self.velo)


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print("Waiting for messages...")

    def stop_robot(sig, frame):
        tb3.vel(0, 0)
        tb3.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(tb3)


if __name__ == "__main__":
    main()
