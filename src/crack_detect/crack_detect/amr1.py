import time
import os
import sys
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from crack_msgs.msg import Obstacle, Danger
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class YOLOImageSubscriber(Node):
    def __init__(self, model):
        super().__init__('yolo_image_subscriber')
        self.group = ReentrantCallbackGroup()
        self.model = model
        self.latest_image = None
        self.depth_image = None
        self.depth_header = None
        self.should_shutdown = False
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.K = None
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10, callback_group=self.group) # 1
        self.obstacle_publisher = self.create_publisher(Obstacle, '/obstacle', 10, callback_group=self.group)  #2
        self.crack_publisher = self.create_publisher(Danger, '/danger', 10, callback_group=self.group)  #3
        self.classNames = model.names if hasattr(model, 'names') else ['Object']
        self.can_publish_car = False     # car publish 가능 여부
        self.car_timer_started = False   # 타이머 중복 방지
        self.car_timer = None  # 타이머 객체 추적용
        self.marker_id = 0


        # 사람 publisher추가
        self.human_publisher = self.create_publisher(Obstacle, '/detected_human',10, callback_group=self.group)

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.subscription = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.rgb_compressed_callback,
            10,callback_group=self.group) # 4

        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,callback_group=self.group) # 5

        self.info_subscription = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            10,callback_group=self.group) # 6

        # ★ 3초 후에 변환 시작 타이머 설정
        self.get_logger().info("TF Tree 안정화 시작. 3초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(3.0, self.start_transform)

    def start_transform(self):
        # 첫 변환 시도
        self.thread = threading.Thread(target=self.detection_loop, daemon=True) # python thread
        self.thread.start()
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")

    def rgb_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB compressed conversion failed: {e}")

    def stop_car_publishing(self):
        self.get_logger().info("5초 경과: car publish 중단")
        self.can_publish_car = False
        self.car_timer_started = False
        if self.car_timer:
            self.car_timer.cancel()  # 타이머 중지
            self.car_timer = None    # 참조 해제

    def calculate_3d_coordinates(self, x, y, z):
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        return X, Y, z

    def detection_loop(self):
        while not self.should_shutdown:
            if self.latest_image is None or self.depth_image is None:
                time.sleep(0.01)
                continue
            
            img = self.latest_image.copy()
            results = self.model.predict(img, stream=True)

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    ux, uy = (x1 + x2) // 2, (y1 + y2) // 2
                    cls = int(box.cls[0]) if box.cls is not None else 0
                    conf = float(box.conf[0]) if box.conf is not None else 0.0
                    class_name = self.classNames[cls]

                    # 깊이 이미지 범위 체크
                    if 0 <= uy < self.depth_image.shape[0] and 0 <= ux < self.depth_image.shape[1]:
                        z = float(self.depth_image[uy, ux]) / 1000.0  # mm → m

                        if z == 0.0 or np.isnan(z):
                            continue  # 잘못된 깊이 데이터는 무시

                        # 3D 좌표 계산
                        X, Y, Z = self.calculate_3d_coordinates(ux, uy, z)

                        # 좌표 변환
                        pt = PointStamped()
                        pt.header.frame_id = self.depth_header.frame_id
                        pt.header.stamp = self.depth_header.stamp
                        pt.point.x = X
                        pt.point.y = Y
                        pt.point.z = Z

                        try:
                            pt_map = self.tf_buffer.transform(
                                pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5)
                            )

                            # 1. 모든 객체 로그 출력 (좌표 포함)
                            self.get_logger().info(
                                f"[{class_name}] Pos: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}, dis: {z:.2f})"
                            )

                            # 2. crack일 때
                            if class_name.lower() == 'crack' :
                                X1, Y1, Z1 = self.calculate_3d_coordinates(x1, y1, z)
                                X2, Y2, Z2 = self.calculate_3d_coordinates(x2, y2, z)
                                size = np.sqrt((X1 - X2) ** 2 + (Y1 - Y2) ** 2 )
                                self.get_logger().info(f"Bounding box 3D width (기존 코드 (x y z)): {size:.2f}")

                                # /danger 메시지 발행
                                danger = Danger()
                                danger.x = round(pt_map.point.x, 2)
                                danger.y = round(pt_map.point.y, 2)
                                danger.z = round(z, 1)
                                danger.size = round(size, 2)

                                ## size는 직접 쟤보면서 확인해봐야 할 것 같음!
                                if size >= 0.15:
                                    self.get_logger().info(
                                    f" Danger! Width {(size-0.15):.2f} over !" )
                                    danger.danger_sense = 1 # 위험
                                    self.crack_publisher.publish(danger)

                                else:
                                    self.get_logger().info(
                                    f" Safe !" )
                                    danger = Danger()
                                    danger.danger_sense = 0 # 안전
                                    self.crack_publisher.publish(danger)

                            # Marker 및 /obstacle 메시지 발행
                            obstacle = Obstacle()
                            obstacle.class_name = class_name
                            obstacle.obstacle_distance = z
                            obstacle.x = round(pt_map.point.x, 2)
                            obstacle.y = round(pt_map.point.y, 2)
                            obstacle.z = round(pt_map.point.z, 2)
                            self.publish_marker(obstacle, pt_map.point.x, pt_map.point.y, self.marker_id)
                            self.marker_id += 1

                            # # 균열일 경우 계속 정보 전달
                            # if class_name.lower() == 'crack':
                            #     self.crack_publisher.publish(danger)

                            # 사람일 경우 10초만 정보 전달
                            if class_name.lower() == 'car':
                                
                                # 타이머가 아직 시작되지 않았다면 시작
                                if not self.car_timer_started:
                                    self.can_publish_car = True
                                    self.car_timer_started = True
                                    self.car_timer = self.create_timer(5.0, self.stop_car_publishing)

                                # 사람 위치 publisher 추가
                                if self.can_publish_car:
                                    self.human_publisher.publish(obstacle)

                        except Exception as e:
                            self.get_logger().warn(f"TF transform to map failed: {e}")

                    # 시각화용 표시 (좌표 상관없이 항상 표시)
                    label = f"{class_name} {conf:.2f}"
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 이미지 띄우기
            cv2.imshow("YOLO Detection with 3D", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutdown requested via 'q'")
                self.should_shutdown = True
                break

    def publish_marker(self, obstacle, x, y, marker_id):
            marker = Marker()
            # Header 설정
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            
            # 마커의 네임스페이스와 ID 설정 (여러 마커를 구분할 때 사용)
            marker.ns = "basic_shapes"
            marker.id = marker_id
            
            # 마커 타입 설정 (SPHERE)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 마커의 위치와 크기 설정
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # 마커 색상 설정
            if obstacle.class_name == 'crack':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # 투명도 (1.0은 불투명)
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # 투명도 (1.0은 불투명)
            # 생명주기 설정 (0.0은 영구적 표시)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # 마커 메시지 발행
            self.publisher_.publish(marker)
def main():
    model_path = '/home/park/myWorkSpace/src/my_best.pt' # 경로 수정 필요
    rclpy.init()
    model = YOLO(model_path)
    node = YOLOImageSubscriber(model)
    executor = MultiThreadedExecutor(num_threads=3) # callback 5
    try:
        while rclpy.ok() and not node.should_shutdown:
            # rclpy.spin_once(node, timeout_sec=0.05)
            executor.add_node(node)
            executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via Ctrl+C.")
    finally:
        node.should_shutdown = True
        node.thread.join(timeout=1.0)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()