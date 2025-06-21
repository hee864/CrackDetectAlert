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
import threading
import time
import cv2


class GoToPersonAdjusted(Node):
    def __init__(self):
        super().__init__('go_to_person_adjusted')
        self.group = ReentrantCallbackGroup()

        self.navigator = TurtleBot4Navigator(namespace='robot7')
        self.action_client = ActionClient(self, NavigateToPose, '/robot7/navigate_to_pose')
        self.count=0
        self._goal_handle = None
        self.bridge = CvBridge()
        self.goal_sent = False
        self.should_shutdown = False
        self.ready_to_respond = False
        self.gotoperson = False
        self.depth_image = None
        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Obstacle, '/obstacle', self.person_callback, 10)
        self.create_subscription(Image, '/robot7/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot7/oakd/rgb/camera_info', self.camera_info_callback, 10)

        # Services
        self.client=self.create_client(PersonFollow, '/person_follow') #client-asynk 
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait for service')
        self.req=PersonFollow.Request()


        self.reperson_service = self.create_service(PersonFollow, '/reperson_follow', self.handle_reperson_request)#server

        # Initial setup
        if not self.navigator.getDockedStatus():
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped([-0.312,1.48], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        self.waypoints=[
            ([-1.327, 6.742],TurtleBot4Directions.NORTH),
                        ([1.227, 6.774],TurtleBot4Directions.EAST)]
        # Start initial navigation thread
        threading.Thread(target=self.initial_waypoints, daemon=True).start()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def send_goal_to_pose(self, pose):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.goal_sent=True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self._goal_handle = goal_handle
            self._goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal was rejected.")

    def goal_result_callback(self, future):
        result = future.result()
        self.goal_sent = False
        self.get_logger().info(f"Goal result status: {result.status}")
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.ready_to_respond = True
        if self.should_shutdown:
            self.get_logger().info("Task complete. Node shutting down.")
            rclpy.shutdown()

    def person_callback(self, msg: Obstacle):
        if self.depth_image is None or self.K is None:
            self.get_logger().warn("Depth image or camera info not ready. Ignoring person position.")
            return

        h, w = self.depth_image.shape[:2]
        cx, cy = w // 2, h // 2
        z = self.depth_image[cy, cx]
        self.get_logger().info(f"Depth at center: {z:.3f}m")

        if z <= 0.5 and msg.class_name=='car':
            self.get_logger().info("Already within 0.5m of person (via depth). No movement needed.")
            return
        elif z>0.5 and msg.class_name=='car' and self.count==0:
            pose = self.navigator.getPoseStamped([msg.x, msg.y], TurtleBot4Directions.NORTH)
            self.send_goal_to_pose(pose)
            # self.goal_sent = True
            self.should_shutdown = True
            self.gotoperson = True
            self.get_logger().info(f"Sending goal to person location: {[msg.x, msg.y]}")
            self.count+=1
            self.send_succeed()

        if  msg.class_name=='car':
            self.req.x = msg.x # car
            self.req.y = msg.y # car   
            


    def initial_waypoints(self):
        # for wp in [([-1.327, 6.742], Turt4leBot4Directions.NORTH),([1.227, 6.774],TurtleBotDirections.EAST)]:
            
        for position, direction in self.waypoints:
            pose=self.navigator.getPoseStamped(position,direction)
            self.send_goal_to_pose(pose)
            self.goal_sent=True
            while self.goal_sent:
                time.sleep(5.0)
            time.sleep(2.0)


   
    def send_succeed(self):
        self.req.person_detect=True
        
        futures=self.client.call_async(self.req)
        futures.add_done_callback(self.response_callback)
        return futures
    
    def response_callback(self,futures):
        result=futures.result().result
        self.get_logger().info(f"[Service] /person_follow responded with: {result}")
        
        
    def handle_reperson_request(self, request, response): #service server:  #person_detect=False라는 요청을 받고 
        # if not request.person_detect:
        #     self.get_logger().info("Reperson service called with False. Retrying person approach.")
        #     pose = self.navigator.getPoseStamped([request.x, request.y], TurtleBot4Directions.NORTH)
        #     self.send_goal_to_pose(pose)
        #     self.goal_sent = True
        #     while self.goal_sent:
        #         time.sleep(0.5)
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
