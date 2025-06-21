#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.subscription = self.create_subscription(PoseStamped, '/waypoint', self.goal_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result_publisher = self.create_publisher(Bool, '/waypoint_done', 10)
        self._processing = False

    def goal_callback(self, msg: PoseStamped):
        if self._processing:
            self.get_logger().warn("Still processing previous goal.")
            return

        self._processing = True
        self.get_logger().info(f" Moving to waypoint: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose server not available.")
            self._processing = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            self._processing = False
            return

        self._goal_handle = goal_handle
        self._goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        if result.status == 3:
            self.get_logger().info("Goal succeeded.")
            self.result_publisher.publish(Bool(data=True))
        else:
            self.get_logger().warn(f"Goal failed with status: {result.status}")
            self.result_publisher.publish(Bool(data=False))

        self._processing = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
