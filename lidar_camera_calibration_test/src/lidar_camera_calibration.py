import cv2
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
import threading
import time
import std_msgs.msg
import struct
import random  # random 모듈 추가

class LidarCameraCalibration:
    def __init__(self):
        rospy.loginfo("Initializing LidarCameraCalibration node")
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.lidar_callback)
        self.camera_sub = rospy.Subscriber('/rotated_image', Image, self.camera_callback)
        self.lidar_data = None
        self.camera_data = None
        self.overlay_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.transform_pub = rospy.Publisher('/lidar_camera_transform', TransformStamped, queue_size=10)
        self.image_pub = rospy.Publisher('/overlay_image', Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher('/overlay_pointcloud', PointCloud2, queue_size=1)

        self.camera_matrix = np.array([[654.424483, 0.000000, 507.498424],
                                       [0.000000, 652.746467, 301.958835],
                                       [0.000000, 0.000000, 1.000000]])
        self.dist_coeffs = np.array([-0.370183, 0.124689, -0.001381, -0.002182, 0.000000])

        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()

        self.last_lidar_update_time = time.time()
        self.lidar_update_interval = 0.5  # 0.5초마다 업데이트

    def display_loop(self):
        while not rospy.is_shutdown():
            if self.overlay_image is not None:
                cv2.imshow('Image with Lidar Points', self.overlay_image)
                cv2.waitKey(1)
        cv2.destroyAllWindows()

    def lidar_callback(self, data):
        current_time = time.time()
        if current_time - self.last_lidar_update_time < self.lidar_update_interval:
            return
        self.last_lidar_update_time = current_time

        try:
            self.lidar_data = data
            self.perform_calibration()
        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")

    def camera_callback(self, data):
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.perform_calibration()
        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {e}")

    def perform_calibration(self):
        if self.lidar_data is None or self.camera_data is None:
            return

        try:
            points = np.array(list(pc2.read_points(self.lidar_data, skip_nans=True, field_names=("x", "y", "z", "intensity"))))

            if points.shape[1] == 4:
                xyz = points[:, :3]
                intensities = points[:, 3]
            else:
                xyz = points
                intensities = np.zeros(points.shape[0])

            try:
                trans = self.tf_buffer.lookup_transform('os_sensor', 'os_lidar', rospy.Time(0), rospy.Duration(1.0))
                rotation_matrix = tf_conversions.transformations.quaternion_matrix([
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ])[:3, :3]
                translation_vector = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                transformation_matrix[:3, 3] = translation_vector

                self.publish_transform(rotation_matrix, translation_vector)

                self.overlay_points_on_image(xyz, transformation_matrix, intensities)

                if self.overlay_image is not None:
                    image_msg = self.bridge.cv2_to_imgmsg(self.overlay_image, "bgr8")
                    self.image_pub.publish(image_msg)
            except Exception as e:
                rospy.logerr(f"Error in getting transform: {e}")
        except Exception as e:
            rospy.logerr(f"Error in perform_calibration: {e}")

    def intensities_to_rgb(self, intensities):
        min_intensity = np.min(intensities)
        max_intensity = np.max(intensities)

        norm_intensities = (intensities - min_intensity) / (max_intensity - min_intensity) * 255
        norm_intensities = norm_intensities.astype(np.uint8)

        colormap = cv2.applyColorMap(norm_intensities.reshape(-1, 1), cv2.COLORMAP_JET)
        return colormap[:, 0, :]

    def publish_transform(self, rotation_matrix, translation_vector):
        try:
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "os_sensor"
            t.child_frame_id = "os_lidar"

            t.transform.translation.x = translation_vector[0]
            t.transform.translation.y = translation_vector[1]
            t.transform.translation.z = translation_vector[2]

            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = translation_vector.flatten()
            quaternion = tf_conversions.transformations.quaternion_from_matrix(transform_matrix)

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            self.transform_pub.publish(t)
        except Exception as e:
            rospy.logerr(f"Error in publish_transform: {e}")

    def overlay_points_on_image(self, points, transformation_matrix, intensities):
        try:
            # X축으로 -90도 회전
            rotation_x_90 = np.array([
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])
            rotation_z_270 = np.array([
                [0, 1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            tttt = np.array([
                [1, 0, 0, -0.4],
                [0, 1, 0, 0],
                [0, 0, 1, 0.1],
                [0, 0, 0, 1]
            ])
            # 변환후 -x가 앞임

            # 변환 행렬 적용
            transformation_matrix = transformation_matrix.dot(rotation_x_90).dot(rotation_z_270).dot(tttt)

            points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))

            transformed_points_homogeneous = points_homogeneous.dot(transformation_matrix.T)
            transformed_points = transformed_points_homogeneous[:, :3] / transformed_points_homogeneous[:, 3][:, np.newaxis]

            # Z 좌표가 1.5에서 2.5 사이인 포인트와 X 좌표가 -0.8에서 0.7 사이인 포인트 선택
            front_points_idx = (1.5 < transformed_points[:, 2]) & (transformed_points[:, 2] < 2.5) & (-0.8 < transformed_points[:, 0]) & (transformed_points[:, 0] < 0.7)
            front_points = transformed_points[front_points_idx]
            front_intensities = intensities[front_points_idx]

            # 필터링된 포인트에 대해 강도 값을 정규화하고 컬러맵을 적용
            front_colors = self.intensities_to_rgb(front_intensities)

            image_points, _ = cv2.projectPoints(front_points, np.zeros((3, 1)), np.zeros((3, 1)), self.camera_matrix, self.dist_coeffs)

            self.overlay_image = self.camera_data.copy()
            valid_points = 0
            invalid_points = 0

            # 새로운 포인트 클라우드 메시지를 위한 리스트
            new_points = []

            for idx, (point, color) in enumerate(zip(image_points, front_colors)):
                x, y = point.ravel()
                if (0 <= x < self.camera_data.shape[1]) and (0 <= y < self.camera_data.shape[0]):
                    bgr_color = tuple(map(int, color))  # Ensure color is in the correct format
                    cv2.circle(self.overlay_image, (int(x), int(y)), 1, bgr_color, -1)
                    valid_points += 1
                    # 이미지의 RGB 값을 가져옴
                    b, g, r = self.camera_data[int(y), int(x)]  # OpenCV에서 BGR 순서
                    # 포인트 클라우드 메시지에 추가
                    new_points.append([front_points[idx, 0], front_points[idx, 1], front_points[idx, 2], r, g, b])
                else:
                    invalid_points += 1

            # 점의 수를 줄이기 위해 샘플링
            sample_ratio = 0.5  # 퍼블리시할 점의 비율 (0.1은 10%)
            new_points = random.sample(new_points, int(len(new_points) * sample_ratio))

            rospy.loginfo(f"Valid points: {valid_points}, Invalid points: {invalid_points}, Sampled points: {len(new_points)}")

            # 새로운 포인트 클라우드 메시지 생성
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'os_sensor'
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1)
            ]
            point_cloud = self.create_cloud_xyzrgb(header, new_points)

            self.pointcloud_pub.publish(point_cloud)

        except Exception as e:
            rospy.logerr(f"Error in overlay_points_on_image: {e}")

    def create_cloud_xyzrgb(self, header, points):
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        cloud_data = []

        for point in points:
            x, y, z, r, g, b = point
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            cloud_data.append([x, y, z, rgb])

        return pc2.create_cloud(header, fields, cloud_data)

if __name__ == "__main__":
    rospy.init_node('lidar_camera_calibration')
    calibration = LidarCameraCalibration()
    rospy.spin()
