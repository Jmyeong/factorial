import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
import cv2
import numpy as np
from cv_bridge import CvBridge

class GetRange:
    def __init__(self, frame):
        self.frame = frame
        self.undistorted = None
        self.points = None
        self.left_angle = None
        self.right_angle = None
        self.qr_angle = None
        self.corrected_angle = None
        self.result = None
        self.bug = None
        self.check = None

    def undistort_image(self):
        camera_matrix = np.array([[1.42485632e+03, 0.00000000e+00, 8.51774883e+02],
                                  [0.00000000e+00, 1.42300125e+03, 5.59285205e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        dist_coeffs = np.array([[-0.37406784, 0.14538707, -0.00168621, 0.0006224, -0.03011034]])

        h, w = self.frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

        self.undistorted = cv2.undistort(self.frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
        self.undistorted = self.undistorted[170:910, 80:1785]

    def qr_detection(self):
        self.undistort_image()
        qr_detector = cv2.QRCodeDetector()
        retval, points, _ = qr_detector.detectAndDecode(self.undistorted)

        if retval:
            points = points.astype(np.int32)
            points = points.reshape((-1, 1, 2))
            self.check = cv2.polylines(self.undistorted, [points], True, (0, 255, 0), 2)
            self.points = points
            print("QR 코드 감지 성공")
        else:
            self.points = None
            self.check = self.undistorted
            print("QR 코드 감지 실패")

    def get_angle(self):
        self.qr_detection()

        if self.points is not None and len(self.points) >= 4:
            left_x = self.points[3][0][0]
            right_x = self.points[2][0][0]
            center_x = np.mean(self.points[:, 0, 0])

            fov = 112
            image_offset = 18.0
            real_offset = -3.8
            self.qr_angle = (center_x - 932.5) / self.undistorted.shape[1] * fov
            self.corrected_angle = self.qr_angle + real_offset + image_offset
            self.left_angle = (left_x - 932.5) / self.undistorted.shape[1] * fov + real_offset + image_offset
            self.right_angle = (right_x - 932.5) / self.undistorted.shape[1] * fov + real_offset + image_offset
            print(f"계산된 각도: Center: {self.qr_angle}, Left: {self.left_angle}, Right: {self.right_angle}")
        else:
            return None

class QRCamera(Node):
    def __init__(self):
        super().__init__('qr_camera')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.camera = cv2.VideoCapture(5)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.check_pub = self.create_publisher(Image, '/camera/qr_cam', 20)
        self.ranged_pub = self.create_publisher(LaserScan, '/filtered', qos)
        self.laserscan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.frame = None
        self.qr = None
        self.bridge = CvBridge()

    def set_frame(self):
        ret, frame = self.camera.read()
        if ret:
            self.frame = frame
            self.qr = GetRange(frame)
            self.qr.qr_detection()

            check_msg = self.bridge.cv2_to_imgmsg(self.qr.check, encoding='bgr8')
            self.check_pub.publish(check_msg)
        else:
            print("카메라에서 프레임을 읽을 수 없습니다.")

    def lidar_callback(self, lidar_msg):
        self.set_frame()
        
        if self.frame is not None:
            self.qr.qr_detection()
            
            if self.qr.points is not None:
                self.qr.get_angle()

                new_minang = np.radians(self.qr.left_angle)
                new_maxang = np.radians(self.qr.right_angle)

                filtered_ranges = []

                for i, distance in enumerate(lidar_msg.ranges):
                    angle_rad = lidar_msg.angle_min + i * lidar_msg.angle_increment
                    if new_minang <= angle_rad <= new_maxang:
                        filtered_ranges.append(float(distance))
                
                if filtered_ranges:
                    new_scan = LaserScan()
                    new_scan.header = lidar_msg.header
                    new_scan.angle_min = new_minang
                    new_scan.angle_max = new_maxang
                    new_scan.angle_increment = lidar_msg.angle_increment
                    new_scan.time_increment = lidar_msg.time_increment
                    new_scan.scan_time = lidar_msg.scan_time
                    new_scan.range_min = lidar_msg.range_min
                    new_scan.range_max = lidar_msg.range_max
                    new_scan.ranges = filtered_ranges
                    new_scan.intensities = []
            
                    self.ranged_pub.publish(new_scan)
                else:
                    print("지정된 각도 내에서 범위를 찾을 수 없습니다.")
            else:
                print("QR 코드 각도를 계산할 수 없습니다.")
        else:
            print("프레임을 처리할 수 없습니다.")

def main(args=None):
    rclpy.init(args=args)
    qr_camera = QRCamera()
    rclpy.spin(qr_camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
