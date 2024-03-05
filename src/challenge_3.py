import signal
import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum
from transforms3d.euler import quat2euler

import time
import math


class State(Enum):
    GO = 1
    STOP = 2
    ROTATE_90_DEGREE_CLOCKWISE = 3
    FINAL_GO = 4


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

        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 1
        )

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.velo = 50
        self.ROBOT_WIDTH = 0.281
        self.tolerance = 0.17  # Ori value 0.17
        self.rotation_buffer = 0.37
        self.collect_values = True
        self.scan_values = []
        self.error = self.ROBOT_WIDTH + self.tolerance
        self.state = State.GO

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
        print(f"State: {self.state}")
        if self.collect_values:
            self.current_values = msg
            self.initial_yaw = self.angles_in_rad[2]
            self.initial_x = self.position.x
            self.initial_y = self.position.y
            self.collect_values = False
        self.latest_scan_data = msg

        self.distance_moved = math.sqrt(
            (self.position.x - self.initial_x) ** 2
            + (self.position.y - self.initial_y) ** 2
        )

        if self.state == State.GO:
            self.go(msg)
        elif self.state == State.STOP:
            self.vel(0, 0)
            time.sleep(0.5)
            self.state = State.ROTATE_90_DEGREE_CLOCKWISE
        elif self.state == State.FINAL_GO:
            self.go(msg)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        pos_x = self.position.x
        pos_y = self.position.y
        pos_z = self.position.z

        orientation = msg.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # print(f"orientation x: {x}")
        self.angles_in_rad = quat2euler([w, x, y, z])
        yaw = self.angles_in_rad[2]
        pitch = self.angles_in_rad[1]  # Not actually needed
        roll = self.angles_in_rad[0]  # Not actually needed
        # Initialized position
        # yaw: 1.5708002582125502
        # pitch: -0.006185335875932426
        # roll: 3.141509317213178
        # print(f"yaw: {yaw}, pitch: {pitch}, roll: {roll}")
        if self.state == State.ROTATE_90_DEGREE_CLOCKWISE:
            self.vel(0, -10)
            self.rotation = self.initial_yaw - yaw
            print(f"Rotation: {self.rotation}")
            if yaw < (
                self.initial_yaw - (math.pi / 2)
            ):  # To be solve: Get the initial yaw values
                self.vel(0, 0)
                self.state = State.FINAL_GO
                self.collect_values = True

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

        print(
            f"Current values: {self.current_values.ranges[0]}, Avg Dist: {avg_distance}, Distance Moved: {self.distance_moved}"
        )
        if State.GO or State.FINAL_GO:
            self.vel(40, 0)
            print(f"Linear Velocity: {self.lin_vel_percent}")
        if self.distance_moved >= 0.15:
            self.vel(0, 0)
            print(f"Linear Velocity: {self.lin_vel_percent}")
            if self.state == State.GO:
                self.state = State.STOP
        elif self.state == State.GO:
            if avg_distance < self.rotation_buffer:
                self.vel(0, 0)
                print(f"Linear Velocity: {self.lin_vel_percent}")
                self.state = State.STOP


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
