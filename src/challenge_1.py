import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


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
        self.tolerance = 0.2

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

        error = self.ROBOT_WIDTH + self.tolerance

        while rclpy.ok():  # To check whether the nodes are running and healthy
            if msg.ranges[0] > error:
                self.update_velocity("acceleration")
                break
            else:
                self.update_velocity("deceleration")
                break

    def update_velocity(self, mode: str):
        """
        Update the velocity of the robot.

        :param mode: To instruct the robot whether it should accelerate or decelerate.
        """
        if mode == "acceleration":
            if self.velo != 0:
                self.velo = self.velo * 1.15
                if self.velo >= 100:
                    self.velo = 100
            else:
                self.velo = 20
        elif mode == "deceleration":
            if self.velo > 10:
                self.velo = self.velo * 0.85
            else:
                self.velo = 0
        self.vel(self.velo)
        print("Info:", self.velo)


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print("waiting for messages...")

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
