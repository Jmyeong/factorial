import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from cv_bridge import CvBridge

class GaussianMeancalc:
    def __init__(self, range):
        self.ranges = np.array(range)
        self.data = self.ranges
        self.calc_result = None
    
    def gaussian_mean(self):
        self.data = self.data[~np.isnan(self.ranges) & np.isfinite(self.ranges)]

        if len(self.data) > 0:
             z_scores = (self.data - np.mean(self.data)) / np.std(self.data)
             z_threshold = 3.0
             self.data = self.data[np.abs(z_scores) < z_threshold]
             mean_value = float(np.mean(self.data))
             self.calc_result = mean_value
        else:
            pass

class CaculateCoord:
    def __init__(self, point, qr_angle, z_coord):
        self.point_z = z_coord
        self.point = point
        self.calculated_coord = None
        self.coord_3d = None
        self.qr_angle = qr_angle
    
    def convert_2d_to_3d(self):

        image_x = sel
        f.point[0]
        image_y = self.point[1]
        


        
    
    def convert_3d_to_3d(self):


class GlobalCoord(Node):
    super().__init__('global_coordinate'):
        self.estimated = None
        self.qr_angle = None
        self.points = None
        self.qrcoord = None
        self.decoded = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filtered_sub = self.create_subscription(LaserScan, '/filtered', self.filtered_callback, qos)
        self.qr_info_sub = self.create_subscription(LaserScan, '/coord_calc', self.qr_info_callback, qos)

    def filtered_callback(self, filtered_msg):
        filtermsg = filtered_msg.ranges
        filtered = GaussianMeancalc(filtermsg)
        self.estimated = filtered.calc_result
    
    def qr_info_callback(self, qr_info_msg):
        center_x = qr_info_msg.ranges[0]
        center_y = qr_info_mgs.ranges[1]
        self.points = [center_x, center_y]
        self.decoded = qr_info_msg.ranges[2]
        self.qr_angle = qr_info_msg.ranges[3]
    
    def global_coord(self):
        cvt = CaculateCoord(self.points, self.angle, self.estimated)
        coord_3d=cvt.coord_3d


        




def main(args=None):
    rclpy.init(args=args)
    global_coordinate = GlobalCoord()
    rclpy.spin(global_coordinate)
    global_coordinate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        
        
        
