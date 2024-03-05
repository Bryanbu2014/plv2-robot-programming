import signal
import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler
from nav_msgs.msg import Odometry


import time
import math

FRONT = 0
BACK = 180
LEFT = 90
RIGHT = -90


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
