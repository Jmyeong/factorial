import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from pyzbar.pyzbar import decode
from std_msgs.msg import Float64

class QRPublisher(Node):
    def __init__(self):
        super().__init__('qr_publisher')
        self.KNOWN_WIDTH = 3.74016  # QR 코드의 알려진 너비 (인치 단위)
        self.publisher_distance = self.create_publisher(Float64, 'qr_distance', 10)
        self.cmtx, self.dist = self.read_camera_parameters()  # Camera parameters are loaded if needed

    def read_camera_parameters(self, filepath='/home/yjchng/ros2_ws/src/qr_localization/camera_param/calibration_result_brio.dat'):
        with open(filepath, 'r') as inf:
            # Ignore first line
            inf.readline()
            cmtx = [list(map(float, inf.readline().split())) for _ in range(3)]
            inf.readline()  # Ignore line that says "distortion"
            dist = [list(map(float, inf.readline().split()))]
        return np.array(cmtx), np.array(dist)

    def find_qr_code(self, image):
        # 이미지에서 QR 코드를 찾습니다.
        decoded_objects = decode(image)
        for obj in decoded_objects:
            if obj.type == 'QRCODE':
                # QR 코드의 사각형 정보를 가져옵니다.
                points = obj.polygon
                # 너비 계산
                if len(points) > 1:
                    width = max([points[i].x - points[i-1].x for i in range(1, len(points))], default=0)
                    return width, points  # 너비와 점의 좌표를 반환
        return None, None

    def distance_to_camera(self, knownWidth, focalLength, perWidth):
        # 카메라로부터의 거리를 계산합니다.
        return (knownWidth * focalLength) / perWidth
    
    def publish_distance(self, distance):
        msg = Float64()
        msg.data = distance
        self.publisher_distance.publish(msg)

    def run(self):
        cap = cv.VideoCapture(6)
        focalLength = None  # 초점 거리를 None으로 초기화

        while True:
            ret, img = cap.read()
            if not ret:
                break
            width, points = self.find_qr_code(img)
            if width:
                if focalLength is None:  # 초점 거리가 설정되지 않았다면
                    focalLength = width * 39.37 / self.KNOWN_WIDTH  # 기본 거리를 사용하여 초점 거리 계산
                inches = self.distance_to_camera(self.KNOWN_WIDTH, focalLength, width)
                self.publish_distance(inches * 0.0254) # 거리를 미터로 변환하여 발행
                if points:
                    points = np.array([p for p in points], np.int32)
                    points = points.reshape((-1, 1, 2))
                    cv.polylines(img, [points], True, (0, 255, 0), 2)
                    cv.putText(img, f"{inches * 0.0254:.2f} meters", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.imshow('QR Code Scanner', img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    qr_publisher = QRPublisher()
    qr_publisher.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
