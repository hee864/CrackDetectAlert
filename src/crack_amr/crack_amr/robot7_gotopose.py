import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from crack_msgs.msg import Obstacle
from crack_msgs.srv import PersonFollow
import tf2_ros
import numpy as np


class GoToPersonAdjusted(Node):
    def __init__(self):
        super().__init__('go_to_person_adjusted')
        self.group = ReentrantCallbackGroup()

        self.navigator = TurtleBot4Navigator(namespace='robot7')
        self.bridge = CvBridge()
        self.depth_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.should_shutdown = False
        self.count = 0

        # Subscriptions
        self.create_subscription(Obstacle, '/obstacle', self.person_callback, 10)
        self.create_subscription(Image, '/robot7/oakd/stereo/image_raw', self.depth_callback, 10)

        # Services
        self.client = self.create_client(PersonFollow, '/person_follow')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /person_follow service...')
        self.req = PersonFollow.Request()
        self.reperson_service = self.create_service(PersonFollow, '/reperson_follow', self.handle_reperson_request)

        # Initialize robot pose
        if not self.navigator.getDockedStatus():
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped([-0.312, 1.48], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # Start waypoints
        self.run_waypoints()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def person_callback(self, msg: Obstacle):
        if self.depth_image is None:
            self.get_logger().warn("Depth image not ready. Ignoring obstacle.")
            return

        h, w = self.depth_image.shape[:2]
        cx, cy = w // 2, h // 2
        z = self.depth_image[cy, cx]
        self.get_logger().info(f"Depth at center: {z:.3f}m")

        if msg.class_name != 'car':
            return

        self.req.x = msg.x
        self.req.y = msg.y

        if z <= 0.5:
            self.get_logger().info("Already within 0.5m. No movement needed.")
            return

        if self.count == 0:
            pose = self.navigator.getPoseStamped([msg.x, msg.y], TurtleBot4Directions.NORTH)
            self.navigator.goToPose(pose)
            self.count += 1
            self.should_shutdown = True
            self.send_succeed()

    def run_waypoints(self):
        waypoints = [
            ([-1.327, 6.742], TurtleBot4Directions.NORTH),
            ([1.227, 6.774], TurtleBot4Directions.EAST)
        ]
        goal_poses = [
            self.navigator.getPoseStamped(pos, dir)
            for pos, dir in waypoints
        ]
        self.navigator.startFollowWaypoints(goal_poses)

    def send_succeed(self):
        self.req.person_detect = True
        future = self.client.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"[Service] /person_follow responded with: {result}")

    def handle_reperson_request(self, request, response):
        response.result = "True"
        return response


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
