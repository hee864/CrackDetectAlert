import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from crack_msgs.msg import Obstacle
from crack_msgs.srv import PersonFollow
import tf2_ros
import numpy as np
import time


class GoToPersonAdjusted(Node):
    def __init__(self):
        super().__init__('go_to_person_adjusted')
        self.group = ReentrantCallbackGroup()

        self.navigator = TurtleBot4Navigator(namespace='robot7')
        self.action_client = ActionClient(self, NavigateToPose, '/robot7/navigate_to_pose')
        self.bridge = CvBridge()
        # self.depth_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_sent = False
        self.should_shutdown = False
        self.count = 0

        # Subscriptions
        self.create_subscription(Obstacle, '/obstacle', self.person_callback, 10)
        # self.create_subscription(Image, '/robot7/oakd/stereo/image_raw', self.depth_callback, 10)

        # # Services
        # self.client = self.create_client(PersonFollow, '/person_follow')
        # # while not self.client.wait_for_service(timeout_sec=1.0):
        # #     self.get_logger().info('Waiting for /person_follow service...')
        # self.req = PersonFollow.Request()

        # self.reperson_service = self.create_service(PersonFollow, '/reperson_follow', self.handle_reperson_request)

        # Initialize robot pose
        # if not self.navigator.getDockedStatus():
        #     self.navigator.dock()
        # # initial_pose = self.navigator.getPoseStamped([-0.312, 1.48], TurtleBot4Directions.NORTH)
        # # self.navigator.setInitialPose(initial_pose)
        # self.navigator.waitUntilNav2Active()
        # self.navigator.undock()

        # Waypoint list
        # self.waypoints = [
        #     ([-0.457, 5.28], TurtleBot4Directions.NORTH),
        #     ([0.417, 5.28], TurtleBot4Directions.EAST)
        # ]

        # Start waypoint navigation
        self.run_waypoints()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def person_callback(self, msg: Obstacle):
        # if self.depth_image is None:
        #     self.get_logger().warn("Depth image not ready. Ignoring obstacle.")
        #     return

        # h, w = self.depth_image.shape[:2]
        # cx, cy = w // 2, h // 2
        # z = self.depth_image[cy, cx]
        # self.get_logger().info(f"Depth at center: {z:.3f}m")

        if msg.class_name != 'car':
            return

        self.req.x = msg.x
        self.req.y = msg.y

        # if z <= 0.5:
        #     self.get_logger().info("Already within 0.5m. No movement needed.")
        #     return

        if self.count == 0:
            pose = self.navigator.getPoseStamped([msg.x, msg.y], TurtleBot4Directions.NORTH)
            self.send_goal_to_pose(pose)
            self.count += 1
            self.should_shutdown = True
            self.send_succeed()

    # def send_goal_to_pose(self, pose: PoseStamped):
    #     if not self.action_client.wait_for_server(timeout_sec=2.0):
    #         self.get_logger().error("Action server not available.")
    #         return

    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose = pose
    #     future = self.action_client.send_goal_async(goal_msg)
    #     future.add_done_callback(self.goal_response_callback)
    #     self.goal_sent = True

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if goal_handle.accepted:
    #         goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
    #     else:
    #         self.get_logger().warn("Goal was rejected.")

    # def goal_result_callback(self, future):
    #     result = future.result()
    #     self.goal_sent = False
    #     self.get_logger().info(f"Goal result status: {result.status}")
    #     if self.should_shutdown:
    #         self.get_logger().info("Shutting down node.")
    #         rclpy.shutdown()

    def run_waypoints(self):
        goal_pose = []
    
        goal_pose.append(self.navigator.getPoseStamped([-0.417,5.28],TurtleBot4Directions.WEST))#4
        goal_pose.append(self.navigator.getPoseStamped([-0.457, 5.28],TurtleBot4Directions.NORTH))#7
    
        self.navigator.startFollowWaypoints(goal_pose)


    # def send_succeed(self):
    #     self.req.person_detect = True
    #     future = self.client.call_async(self.req)
    #     future.add_done_callback(self.response_callback)

    # def response_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"[Service] /person_follow responded with: {result}")

    # def handle_reperson_request(self, request, response):
    #     response.result = "True"
    #     return response


def main():
    rclpy.init()
    node = GoToPersonAdjusted()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.should_shutdown = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
