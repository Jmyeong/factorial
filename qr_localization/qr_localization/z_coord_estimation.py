import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
# from laser_geometry import LaserProjection
# import point_cloud2 as pc2
# import laser_geometry
from collections import namedtuple
import ctypes
import math
import struct
# test

_DATATYPES = {
    PointField.INT8:    ('b', 1),
    PointField.UINT8:   ('B', 1),
    PointField.INT16:   ('h', 2),
    PointField.UINT16:  ('H', 2),
    PointField.INT32:   ('i', 4),
    PointField.UINT32:  ('I', 4),
    PointField.FLOAT32: ('f', 4),
    PointField.FLOAT64: ('d', 8)
}

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def read_points_list(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'

    if field_names is None:
        field_names = [f.name for f in cloud.fields]

    Point = namedtuple("Point", field_names)

    return [Point._make(l) for l in read_points(cloud, field_names, skip_nans, uvs)]

def create_cloud(header, fields, points):
    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=cloud_struct.size,
        row_step=cloud_struct.size * len(points),
        data=buff.raw
    )

def create_cloud_xyz32(header, points):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    return create_cloud(header, fields, points)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt
######################################################################

class LaserProjection:
    """
    A class to Project Laser Scan

    This class will project laser scans into point clouds. It caches
    unit vectors between runs (provided the angular resolution of
    your scanner is not changing) to avoid excess computation.

    By default all range values less than the scanner min_range or
    greater than the scanner max_range are removed from the generated
    point cloud, as these are assumed to be invalid.

    If it is important to preserve a mapping between the index of
    range values and points in the cloud, the recommended approach is to
    pre-filter your laser scan message to meet the requirement that all
    ranges are between min and max_range.

    The generate PointClouds have a number of channels which can be enabled
    through the use of ChannelOption.
    - ChannelOption.INTENSITY - Create a channel named "intensities" with the
    intensity of the return for each point.
    - ChannelOption.INDEX     - Create a channel named "index" containing the
    index from the original array for each point.
    - ChannelOption.DISTANCE  - Create a channel named "distance" containing
    the distance from the laser to each point.
    - ChannelOption.TIMESTAMP - Create a channel named "stamps" containing the
    specific timestamp at which each point was measured.
    """

    LASER_SCAN_INVALID   = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE      = 0x00 # Enable no channels
        INTENSITY = 0x01 # Enable "intensities" channel
        INDEX     = 0x02 # Enable "index"       channel
        DISTANCE  = 0x04 # Enable "distances"   channel
        TIMESTAMP = 0x08 # Enable "stamps"      channel
        VIEWPOINT = 0x10 # Enable "viewpoint"   channel
        DEFAULT   = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0

        self.__cos_sin_map = np.array([[]])

    def projectLaser(self, scan_in,
            range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.

        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.

        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)

    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
            self.__angle_max != scan_in.angle_max):
            rclpy.logging.get_logger("project_laser").debug(
                    "No precomputed map given. Computing one.")

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max
            
            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance =  idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
            len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

        cloud_out = create_cloud(scan_in.header, fields, points)

        return cloud_out


class GaussianMeanCalculator(Node):

    def __init__(self):
        super().__init__('gaussian_mean_calculator')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filtered_sub = self.create_subscription(LaserScan, '/filtered', self.filtered_callback, qos)
        self.z_coord_pub = self.create_publisher(LaserScan, '/estimated_Z', 10)
        self.cloud_publish = self.create_publisher(PointCloud2, '/cloud', qos)

        self.data = None
        self.filtered_data = None

    def filtered_callback(self, filtered_msg):
        self.filtered_data = np.array(filtered_msg.ranges)
        self.data = self.filtered_data[~np.isnan(self.filtered_data) & np.isfinite(self.filtered_data)]
        self.cloud = LaserProjection().projectLaser(filtered_msg)
        for point in read_points(self.cloud, field_names=("x", "y"), skip_nans=True):
            x, y = point[:2]
            self.get_logger().info(f'Point (x: {x}, y: {y})')

        if len(self.data) > 0:
             z_scores = (self.data - np.mean(self.data)) / np.std(self.data)
             z_threshold = 3.0
             self.data = self.data[np.abs(z_scores) < z_threshold]
             mean_value = float(np.mean(self.data))
             self.data = [mean_value, mean_value, mean_value]


             z_coord_msg = LaserScan()
             z_coord_msg.header = filtered_msg.header
             z_coord_msg.header.frame_id = 'map'
             z_coord_msg.angle_min = filtered_msg.angle_min
             z_coord_msg.angle_max = filtered_msg.angle_max
             z_coord_msg.angle_increment = filtered_msg.angle_increment
             z_coord_msg.time_increment = filtered_msg.time_increment
             z_coord_msg.scan_time = filtered_msg.scan_time
             z_coord_msg.range_min = filtered_msg.range_min
             z_coord_msg.range_max = filtered_msg.range_max
             z_coord_msg.ranges = self.data

             self.z_coord_pub.publish(z_coord_msg)
             self.cloud_publish.publish(self.cloud)

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
