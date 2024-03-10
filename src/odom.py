import math
import signal

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler

from utils import *


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
        self.transformation = None

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """Publishes linear and angular velocities in percent"""
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
        """Is run whenever a LaserScan msg is received"""
        # show_scan_callback(msg)

    def odom_callback(self, msg):
        show_odom_callback(msg)
        position = msg.pose.pose.position
        self.pos_x = position.x
        self.pos_y = position.y
        self.pos_z = position.z

        # Handle orientation
        orientation = msg.pose.pose.orientation
        ori_w = orientation.w
        ori_x = orientation.x
        ori_y = orientation.y
        ori_z = orientation.z

        self.angles_in_rad = quat2euler([ori_w, ori_x, ori_y, ori_z])
        yaw_in_deg = math.degrees(
            self.angles_in_rad[2]
        )  # Convert yaw from radian to degree

        if self.transformation is None:
            self.odom_start_to_odom, self.odom_to_odom_start = (
                create_transformations_between_odom_start_and_odom(
                    (self.pos_x, self.pos_y), self.angles_in_rad[2]
                )
            )
            self.transformation = "Done"

        if self.transformation == "Done":
            robot_pos = self.odom_to_odom_start((self.pos_x, self.pos_y))
            print(f"Robot Position -> Odom Start: {robot_pos}")
            print(f"Odom: {self.odom_start_to_odom(robot_pos)}")
            print("")


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
