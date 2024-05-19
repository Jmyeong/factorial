import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class GaussianMeanCalculator(Node):
    def __init__(self):
        super().__init__('gaussian_mean_calculator')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filtered_sub = self.create_subscription(LaserScan, '/filtered', self.filtered_callback, qos)
        self.z_coord_pub = self.create_publisher(Float64, '/estimated_Z', 10)
        self.data = None
        self.filtered_data = None

    def filtered_callback(self, filtered_msg):
        self.filtered_data = np.array(filtered_msg.ranges)
        self.data = self.filtered_data[~np.isnan(self.filtered_data) & np.isfinite(self.filtered_data)]

        if len(self.data) > 0:
            z_scores = (self.data - np.mean(self.data)) / np.std(self.data)
            z_threshold = 3.0
            self.data = self.data[np.abs(z_scores) < z_threshold]
            mean_value = float(np.mean(self.data))
            
            z_coord_msg = Float64()
            z_coord_msg.data = mean_value
            self.z_coord_pub.publish(z_coord_msg)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    gaussian_mean_calculator = GaussianMeanCalculator()
    rclpy.spin(gaussian_mean_calculator)
    gaussian_mean_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
