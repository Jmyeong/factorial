import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class GaussianMeanCalculator(Node):

    def __init__(self):
        super().__init__('gaussian_mean_calculator')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filtered_sub = self.create_subscription(LaserScan, '/filtered', self.filtered_callback, qos)
        self.z_coord_pub = self.create_publisher(LaserScan, '/estimated_Z', 10)
        self.log_file = open('/home/yjchng/filtered_data_log.txt', 'w')


    def filtered_callback(self, filtered_msg):
        self.filtered_data = np.array(filtered_msg.ranges)
        self.data = self.filtered_data[~np.isnan(self.filtered_data) & np.isfinite(self.filtered_data)]

        # 받아온 메시지를 파일에 로깅
        self.log_file.write(f'{filtered_msg.header.stamp.sec}.{filtered_msg.header.stamp.nanosec}, ' +
                            ','.join(map(str, filtered_msg.ranges)) + '\n')

        if len(self.data) > 0:
            z_scores = (self.data - np.mean(self.data)) / np.std(self.data)
            z_threshold = 3.0
            self.data = self.data[np.abs(z_scores) < z_threshold]
            mean_value = float(np.mean(self.data))
            self.data = [mean_value, mean_value, mean_value]

            z_coord_msg = LaserScan()
            z_coord_msg.header = filtered_msg.header
            z_coord_msg.angle_min = filtered_msg.angle_min
            z_coord_msg.angle_max = filtered_msg.angle_max
            z_coord_msg.angle_increment = filtered_msg.angle_increment
            z_coord_msg.time_increment = filtered_msg.time_increment
            z_coord_msg.scan_time = filtered_msg.scan_time
            z_coord_msg.range_min = filtered_msg.range_min
            z_coord_msg.range_max = filtered_msg.range_max
            z_coord_msg.ranges = self.data

            self.z_coord_pub.publish(z_coord_msg)
        else:
            pass

    def destroy_node(self):
        self.log_file.close()  # 파일을 닫고 노드를 종료
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gaussian_mean_calculator = GaussianMeanCalculator()
    rclpy.spin(gaussian_mean_calculator)
    gaussian_mean_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
