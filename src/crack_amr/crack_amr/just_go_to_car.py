import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from crack_msgs.msg import Obstacle  
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator



#ros2 topic pub --once /obstacle crack_msgs/msg/Obstacle "{class_name: 'car', obstacle_distance: 1.0, x: -2.16, y: 4.15 ,z: 0.0 }" 
from crack_msgs.srv import PersonFollow
class ObstacleFollower(Node):
    def __init__(self):
        super().__init__('obstacle_follower')
        self.get_logger().info(f"Istart!")

        #이동 명령을 위한 nav2 액션 클라이언트 설정
        self.action_client = ActionClient(self, NavigateToPose, '/robot7/navigate_to_pose')
        self._goal_handle = None

        #장애물 정보를 구독
        self.create_subscription(Obstacle, '/obstacle', self.obstacle_callback, 10)
        # Services- 사람 추적 결과를 전달하기 위한 서비스 클라이언트 생성
        self.client = self.create_client(PersonFollow, '/person_follow')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /person_follow service...')
        # self.req = PersonFollow.Request()
        self.get_logger().info(f"Istart!")


        #터틀봇 전용 내비게이션 도우미 객체 생성
        navigator = TurtleBot4Navigator(namespace='robot7')
        navigator.waitUntilNav2Active()

        #로봇이 따라갈 초기 웨이포인트 (출구 탐색용)
        goal_pose = []        
        goal_pose.append(navigator.getPoseStamped([-0.457,5.28], TurtleBot4Directions.WEST))#4
        goal_pose.append(navigator.getPoseStamped([-0.457,5.28], TurtleBot4Directions.NORTH))#7

        # 웨이포인트 따라가기 시작
        navigator.startFollowWaypoints(goal_pose)


        #재탐색 서비스 서버 등록(재요청 시 응답 반환)
        self.reperson_service = self.create_service(PersonFollow, '/reperson_follow', self.handle_reperson_request)
    
    #Euler각을 Quaternion으로 변환 (ROS Pose 메시지용)
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    #장애물 토픽 콜백함수 (자동화 감지 시 해당좌표로 이동 명령 전송)
    def obstacle_callback(self, msg: Obstacle):

        if msg.class_name != 'car':
            self.get_logger().info(f"Ignoring class: {msg.class_name}")
            return

        self.get_logger().info(f"Received obstacle at x={msg.x}, y={msg.y}, class={msg.class_name}")

        # 이동 목표 좌표 설정
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.euler_to_quaternion(0, 0, 0)

        self.send_goal(pose)


    #목적지 PoseStamped를 이용해 goal 전송
    def send_goal(self, pose: PoseStamped):
        self.get_logger().info("Sending goal to move_base...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        #액션 서버 준비 확인
        self.action_client.wait_for_server()
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('action server not available')
            return 
        

        #목표 위치 전송 및 응답 콜백연결
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)


    #goal 수락 여부 처리 콜백
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'goal response failed')
            return
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle

        #결과 콜백 연결
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)


    #이동 결과에 따른 처리
    def goal_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_ABORTED
            self.get_logger().warn("Goal was aborted.")
        elif result.status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal succeeded.")
            self.send_succeed()
        else:
            self.get_logger().info(f"Goal finished with status: {result.status}")

    #이동 성공 시 서비스 호출 (사람을 찾았다고 신호를 보냄)
    def send_succeed(self):
        self.req.person_detect = True
        future = self.client.call_async(self.req)
        future.add_done_callback(self.response_callback)


    #서비스 응답 로그 출력
    def response_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"[Service] /person_follow responded with: {result}")


    #/reperson_follow 서비스 응답 (True 반환)
    def handle_reperson_request(self, request, response):
        response.result = "True"
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


