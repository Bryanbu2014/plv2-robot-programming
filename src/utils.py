import signal
import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler
from nav_msgs.msg import Odometry
from enum import Enum, auto


import time
import math

FRONT = 0
BACK = 180
LEFT = 90
RIGHT = -90


class State(Enum):
    FORWARD = auto()
    BLOCKED = auto()
    DECIDE_ROTATION = auto()
    ROTATE_LEFT = auto()
    ROTATE_RIGHT = auto()
    STOP = auto()


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_avg_distance(scanned_values, msg):
    if len(scanned_values) < 2:
        scanned_values.append(msg.ranges[0])
        print("Scan values: " + str(scanned_values))
        avg_distance = scanned_values[0]
        return avg_distance

    scanned_values.pop(0)
    scanned_values.append(msg.ranges[0])
    print("Scan values: " + str(scanned_values))

    avg_distance = (scanned_values[0] + scanned_values[1]) / 2
    return avg_distance


def show_scan_callback(msg):
    print()
    print("Distances:")
    print("⬆️ :", msg.ranges[FRONT])
    print("⬇️ :", msg.ranges[BACK])
    print("⬅️ :", msg.ranges[LEFT])
    print("➡️ :", msg.ranges[RIGHT])


def show_odom_callback(msg):
    position = msg.pose.pose.position
    pos_x = position.x
    pos_y = position.y
    pos_z = position.z
    print(f"x: {pos_x}")
    print(f"y: {pos_y}")
    print(f"z: {pos_z}")

    orientation = msg.pose.pose.orientation
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    # print(f"orientation x: {x}")
    angles_in_rad = quat2euler([w, x, y, z])
    roll = math.degrees(angles_in_rad[0])
    pitch = math.degrees(angles_in_rad[1])
    yaw = math.degrees(angles_in_rad[2])
    print(f"Yaw: {yaw}")
    print(f"Pitch: {pitch}")
    print(f"Roll: {roll}")
    print("")


def show_info(state, lin_vel_percent, ang_vel_percent, rotation=None):
    print(f"Current State: {state}")
    print(f"Lin and Ang Velocity: ({lin_vel_percent}, {ang_vel_percent})")
    if state == State.ROTATE_LEFT or state == State.ROTATE_RIGHT:
        print(f"Rotation: {abs(rotation)}")
    else:
        print(f"Rotation: Not rotating")
    print(f"")


####################################
#      Utils from Prof. Aydos      #
####################################


def rotate(point: tuple[float, float], angle: float):
    from math import cos, sin

    return (
        point[0] * cos(angle) - point[1] * sin(angle),
        point[1] * cos(angle) + point[0] * sin(angle),
    )


def translate(point: tuple[float, float], translation: tuple[float, float]):
    return point[0] + translation[0], point[1] + translation[1]


def create_transformations_between_odom_start_and_odom(
    start_pos_in_odom: tuple[float, float], start_ori_in_odom: float
):
    from math import fmod, pi

    def odom_start_to_odom(point_or_ori: tuple[float, float] | float):
        if isinstance(point_or_ori, tuple):
            point = point_or_ori
            return translate(
                rotate(point, start_ori_in_odom),
                start_pos_in_odom,
            )
        ori = point_or_ori
        return fmod(ori + start_ori_in_odom, pi)

    def odom_to_odom_start(point_or_ori: tuple[float, float] | float):
        # Reverse of `maze_to_odom`
        def minus(point):
            return -point[0], -point[1]

        if isinstance(point_or_ori, tuple):
            point = point_or_ori
            return rotate(
                translate(point, minus(start_pos_in_odom)),
                -start_ori_in_odom,
            )
        ori = point_or_ori

        return fmod(ori - start_ori_in_odom, pi)

    return odom_start_to_odom, odom_to_odom_start
