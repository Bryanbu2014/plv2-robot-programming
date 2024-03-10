import math
import signal
import sys
import time
from enum import Enum, auto

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler

from utils import *


class State(Enum):
    FORWARD = auto()
    BLOCKED = auto()
    DECIDE_ROTATION = auto()
    ROTATE_LEFT = auto()
    ROTATE_RIGHT = auto()
    STOP = auto()


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
        self.ROBOT_WIDTH = 0.281
        self.TOLERANCE = 0.17
        self.ROTATION_BUFFER = 0.37
        self.state = State.FORWARD
        self.scanned_values = None
        self.initial_position = None
        self.initial_orientation = None
        self.current_cell = ()
        self.TARGETED_CELL = (2, 2)
        self.starting_time = None

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

        while self.state != State.DECIDE_ROTATION:
            self.scanned_values = msg
            break

        match self.state:
            case State.FORWARD:
                if self.starting_time is None:
                    self.starting_time = time.time()
                self.forward(msg)
            case State.STOP:
                self.stop()

    def odom_callback(self, msg):
        # Handle position
        position = msg.pose.pose.position
        self.pos_x = position.x
        self.pos_y = position.y
        self.pos_z = position.z

        if self.initial_position is None:
            self.initial_position = position

        # Handle orientation
        orientation = msg.pose.pose.orientation
        ori_w = orientation.w
        ori_x = orientation.x
        ori_y = orientation.y
        ori_z = orientation.z

        if self.initial_orientation is None:
            self.initial_orientation = quat2euler([ori_w, ori_x, ori_y, ori_z])

        self.angles_in_rad = quat2euler([ori_w, ori_x, ori_y, ori_z])
        yaw_in_deg = math.degrees(
            self.angles_in_rad[2]
        )  # Convert yaw from radian to degree

        initial_yaw_in_deg = math.degrees(self.initial_orientation[2])

        rotation = normalize_angle(initial_yaw_in_deg - yaw_in_deg)

        print(f"Current state: {self.state}")
        print(f"Lin and Ang Velocity: ({self.lin_vel_percent}, {self.ang_vel_percent})")
        if self.state == State.ROTATE_LEFT or self.state == State.ROTATE_RIGHT:
            print(f"Rotation: {abs(rotation)}")
        else:
            print(f"Rotation: Not rotating")
        print(f"")

        match self.state:
            case State.BLOCKED:
                self.vel(0, 0)
                self.check_and_update_position()
            case State.DECIDE_ROTATION:
                if self.scanned_values.ranges[RIGHT] > self.scanned_values.ranges[LEFT]:
                    self.state = State.ROTATE_RIGHT
                elif (
                    self.scanned_values.ranges[LEFT] > self.scanned_values.ranges[RIGHT]
                ):
                    self.state = State.ROTATE_LEFT
            case State.ROTATE_RIGHT:
                self.rotate("right", 90, rotation)
            case State.ROTATE_LEFT:
                self.rotate("left", 90, rotation)
            case State.STOP:
                self.stop()

    def forward(self, msg):
        self.vel(50, 0)
        if msg.ranges[FRONT] < (self.ROBOT_WIDTH + self.TOLERANCE):
            self.vel(0, 0)
            self.state = State.BLOCKED

    def rotate(self, direction, angle_threshold, rotation):
        self.ang_vel_percent = -10 if direction == "right" else 10
        self.vel(0, self.ang_vel_percent)

        if abs(rotation) >= angle_threshold:
            self.initial_orientation = None
            self.vel(0, 0)
            self.state = State.FORWARD

    def stop(self):
        self.vel(0, 0)
        current_time = time.time()
        time_elapse = current_time - self.starting_time
        print(f"Time elapsed: {time_elapse}")
        sys.exit(0)

    def check_and_update_position(self):
        x = math.floor(self.pos_x)
        y = math.floor(self.pos_y)
        self.current_cell = (x, y)
        if self.current_cell == self.TARGETED_CELL:
            self.state = State.STOP
        else:
            self.state = State.DECIDE_ROTATION


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
