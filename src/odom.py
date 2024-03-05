import signal

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
from nav_msgs.msg import Odometry


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
        print()
        print("Distances:")
        n = len(msg.ranges)
        print("⬆️ :", msg.ranges[0])
        print("⬇️ :", msg.ranges[n // 2])
        print("⬅️ :", msg.ranges[n // 4])
        print("➡️ :", msg.ranges[-n // 4])

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        pos_x = self.position.x
        pos_y = self.position.y
        pos_z = self.position.z
        print(f"{pos_x}, {pos_y}")

        orientation = msg.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # print(f"orientation x: {x}")
        angles_in_rad = quat2euler([w, x, y, z])
        roll = angles_in_rad[0]
        pitch = angles_in_rad[1]
        yaw = angles_in_rad[2]
        print(f"Yaw: {yaw}")
        print(f"Pitch: {pitch}")
        print(f"Roll: {roll}")


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
