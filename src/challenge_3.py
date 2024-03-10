import math
import signal
import time
from enum import Enum

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler

from utils import normalize_angle


class State(Enum):
    FIRST_GO = 1
    STOP = 2
    ROTATE_90_DEGREES_CLOCKWISE = 3
    SECOND_GO = 4


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
        self.state = State.FIRST_GO
        self.initial_position = None
        self.initial_orientation = None

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
        # print()
        # print("Distances:")
        # print("⬆️ :", msg.ranges[0])
        # print("⬇️ :", msg.ranges[180])
        # print("⬅️ :", msg.ranges[90])
        # print("➡️ :", msg.ranges[-90])

    def odom_callback(self, msg):
        # Handle position
        position = msg.pose.pose.position
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z

        if self.initial_position is None:
            self.initial_position = position
            print(self.initial_position)

        distance_moved = math.sqrt(
            (pos_x - self.initial_position.x) ** 2
            + (pos_y - self.initial_position.y) ** 2
        )

        # Handle orientation
        orientation = msg.pose.pose.orientation
        ori_w = orientation.w
        ori_x = orientation.x
        ori_y = orientation.y
        ori_z = orientation.z

        if self.initial_orientation is None:
            self.initial_orientation = quat2euler([ori_w, ori_x, ori_y, ori_z])
            print(self.initial_orientation)

        self.angles_in_rad = quat2euler([ori_w, ori_x, ori_y, ori_z])
        yaw_in_deg = math.degrees(
            self.angles_in_rad[2]
        )  # Convert yaw from radian to degree

        initial_yaw_in_deg = math.degrees(self.initial_orientation[2])

        rotation = normalize_angle(initial_yaw_in_deg - yaw_in_deg)

        print(f"Lin Velo: {self.lin_vel_percent}")
        print(f"Ang Velo: {self.ang_vel_percent}")
        print(f"Rotation: {rotation}")
        print(f"Distance Moved: {distance_moved}")
        print(f"")

        match self.state:
            case State.FIRST_GO:
                self.vel(20, 0)
                if distance_moved >= 0.15:
                    self.vel(0, 0)
                    time.sleep(0.5)
                    self.state = State.STOP
            case State.STOP:
                self.state = State.ROTATE_90_DEGREES_CLOCKWISE
            case State.ROTATE_90_DEGREES_CLOCKWISE:
                self.vel(0, -20)
                if rotation >= 90:
                    self.vel(0, 0)
                    self.initial_position = None
                    time.sleep(0.5)
                    self.state = State.SECOND_GO
            case State.SECOND_GO:
                if distance_moved >= 0.15:
                    self.vel(0, 0)
                else:
                    self.vel(20, 0)


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
