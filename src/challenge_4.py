import math
import signal
import sys
import time

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
        self.ROBOT_WIDTH = 0.281
        self.TOLERANCE = 0.10
        self.ROTATION_BUFFER = 0.37
        self.state = State.FORWARD
        self.scanned_values = None
        self.initial_position = None
        self.initial_orientation = None
        self.current_cell = ()
        self.CELL_SIZE: tuple[int, int] = ()  # Please fill in the size here
        self.TARGETED_CELL = (self.CELL_SIZE[0] - 1, -self.CELL_SIZE[1] + 1)
        self.starting_time = None
        self.transformation = None

    def vel(self, lin_vel_percent: float, ang_vel_percent: float = 0):
        """
        Publishes linear and angular velocities in percent.

        :param lin_vel_percent: Linear velocity of the robot in percent.
        :param ang_vel_percent: Angular velocity of the robot in percent.
        """

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
        """
        Is run whenever a LaserScan msg is received.
        """

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
        """
        Processes the odometry data to update the robot's position and orientation. It sets initial values if they
        are not already set, handles transformations, and updates the robot's navigation state based on its current
        orientation and position data received from the odometry message.

        :param msg: An odometry message object, typically containing the robot's current position and orientation.
                    The data is used to update the robot's internal state and perform necessary navigation adjustments.
        """

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

        if self.transformation is None:
            self.odom_start_to_odom, self.odom_to_odom_start = (
                create_transformations_between_odom_start_and_odom(
                    (self.pos_x, self.pos_y), self.angles_in_rad[2]
                )
            )
            self.transformation = "Done"

        if self.transformation == "Done":
            self.robot_pos = self.odom_to_odom_start((self.pos_x, self.pos_y))

        initial_yaw_in_deg = math.degrees(self.initial_orientation[2])

        rotation = normalize_angle(initial_yaw_in_deg - yaw_in_deg)

        show_info(
            self.state,
            self.lin_vel_percent,
            self.ang_vel_percent,
            rotation,
        )

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
        """
        Controls the robot's forward movement based on sensor readings. The robot starts moving forward at a set speed.
        If an obstacle is detected within a predefined safety distance (considering the robot's width and an additional tolerance),
        the robot stops and its state is set to BLOCKED.

        :param msg: An object containing sensor data, which includes distance measurements from various directions.
                    The function specifically checks the distance in the 'FRONT' direction to decide on movement control.
        """

        self.vel(70, 0)
        if msg.ranges[FRONT] < (self.ROBOT_WIDTH + self.TOLERANCE):
            self.vel(0, 0)
            self.state = State.BLOCKED

    def rotate(self, direction: str, angle_threshold: float, rotation: float):
        """
        Rotates the robot in a clockwise or anti-clockwise direction. Stops once defined threshold angle is reached.

        :param direction: Direction of the rotation. "right" to rotates clockwise. "left" to rotates anti-clockwise.
        :param angle_threshold: Maximum degree of rotation in degree. E.g. 90 means the rotate 90 degrees.
        :param rotation: Rotated angle.
        """

        self.ang_vel_percent = -30 if direction == "right" else 30
        self.vel(0, self.ang_vel_percent)

        if abs(rotation) >= angle_threshold:
            self.initial_orientation = None
            self.vel(0, 0)
            self.state = State.FORWARD

    def stop(self):
        """
        Stops the robot. Exits the program once the robot reaches the targeted cell.
        """

        self.vel(0, 0)
        current_time = time.time()
        time_elapse = current_time - self.starting_time
        print(f"Time elapsed: {time_elapse}")
        sys.exit(0)

    def check_and_update_position(self):
        """
        Checks and updates the robot's position relative to odom everytime the function is called.
        """

        x = (
            math.floor(self.robot_pos[0])
            if self.robot_pos[0] > 0
            else math.ceil(self.robot_pos[0])
        )
        y = (
            math.floor(self.robot_pos[1])
            if self.robot_pos[1] > 0
            else math.ceil(self.robot_pos[1])
        )
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
