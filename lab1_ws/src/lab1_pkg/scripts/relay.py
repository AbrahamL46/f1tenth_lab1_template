#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__("relay")
        self.sub = self.create_subscription(AckermannDriveStamped, "drive", self.cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, "drive_relay", 10)

    def cb(self, msg_in: AckermannDriveStamped):
        out = AckermannDriveStamped()
        out.header = msg_in.header
        out.drive.speed = msg_in.drive.speed * 3.0
        out.drive.steering_angle = msg_in.drive.steering_angle * 3.0
        self.pub.publish(out)

def main():
    rclpy.init(); rclpy.spin(Relay()); rclpy.shutdown()

if __name__ == "__main__":
    main()