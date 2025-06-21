#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/waypoint', 10)
        self.subscription = self.create_subscription(Bool, '/waypoint_done', self.done_callback, 10)
        self.index = 0
        self.ready = True

        self.waypoints = [
            ([-2.39, 0.143], 1.532),
            ([-1.735, 1.066], 1.523),
            ([-0.822, 2.68], 2.419),
            ([-2.16, 4.15], 1.482),
            ([-2.01, 5.61], -0.043),
            ([-0.457, 5.28], -0.067),
            ([0.392, 0.227], -0.00143),
        ]

        self.timer = self.create_timer(1.0, self.publish_if_ready)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

    def publish_if_ready(self):
        if self.ready and self.index < len(self.waypoints):
            x, y = self.waypoints[self.index][0]
            yaw = self.waypoints[self.index][1]

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y

            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.publisher_.publish(pose)
            self.get_logger().info(f"Sent waypoint {self.index + 1}")
            self.ready = False

    def done_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("✔️ Waypoint completed.")
            self.index += 1
            self.ready = True

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
