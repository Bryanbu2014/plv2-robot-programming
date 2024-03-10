import signal

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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
        self.tolerance = 0.17
        self.emergency_brake = 0.20
        self.scan_values = []

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
        n = len(msg.ranges)
        print("⬆️ :", msg.ranges[0])
        print("⬇️ :", msg.ranges[n // 2])
        print("⬅️ :", msg.ranges[n // 4])
        print("➡️ :", msg.ranges[-n // 4])
        self.latest_scan_data = msg

        if len(self.scan_values) < 2:
            self.scan_values.append(msg.ranges[0])
            print("Scan values: " + str(self.scan_values))
            return

        self.scan_values.pop(0)
        self.scan_values.append(msg.ranges[0])
        print("Scan values: " + str(self.scan_values))

        avg_distance = (self.scan_values[0] + self.scan_values[1]) / 2
        print("Average distance: " + str(avg_distance))

        error = self.ROBOT_WIDTH + self.tolerance

        if avg_distance > error:
            self.update_velocity("acceleration")
        elif avg_distance < self.emergency_brake:
            self.update_velocity("emergency brake")
        else:
            self.update_velocity("deceleration")

    def update_velocity(self, mode: str):
        """
        Update the velocity of the robot.

        :param mode: To instruct the robot whether it should accelerate or decelerate.
        """
        if mode == "acceleration":
            self.velo = self.velo + 20
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
    print("If you cannot kill the process using CTRL+C, use CTRL+\\")

    def stop_robot(sig, frame):
        tb3.vel(0, 0)
        tb3.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(tb3)


if __name__ == "__main__":
    main()
