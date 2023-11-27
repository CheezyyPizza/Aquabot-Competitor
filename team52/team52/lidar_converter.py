import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from team52_interfaces.msg import Obstacles
from team52_interfaces.srv import ConvertToGPS

#>===[ CONVERSION ]===<#
# --- Exponential --- #
def quick_exp(x,e):
    if e < 0:
        return quick_exp(1/x, -e)
    if not e:
        return 1
    exp = quick_exp(x,e//2)
    if e % 2:
        return exp*exp*x
    else:
        return exp*exp

# --- Ints -> Float --- #
def int_to_float(ints):
    bin = ''.join(['{0:08b}'.format(ints[3-i]) for i in range(4)])
    # Sign #
    sign = -1
    if bin[0] == '0':
        sign = 1
    # Exponent #
    exponent = sum([int(bin[i])*quick_exp(2,8-i) for i in range(1,9)])
    e = quick_exp(2, exponent-127)
    # Mantis #
    mantis = 1
    for i in range(23):
        mantis += int(bin[-(i+1)])*quick_exp(2, -23+i)
    # Result #
    return sign * e * mantis

#>===[ RAW TO POINT ]===<#
def get_point(data, offset):
    return [int_to_float([data[offset + dim*4 + i] for i in range(4)]) for dim in range(3)]

#>===[ QUATERNION TO TILTS ]===<#
def quaternion_to_tilts(qw, qx, qy, qz):
    roll = math.tan(math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)))
    pitch = 2 * (qw * qy - qz * qx)
    return [pitch, roll]

#>===[ AVERAGE POINT ]===<#
def average_point(point_list):
    sum = [0,0,0]
    l = len(point_list)
    for point in point_list:
        for i in range(3):
            sum[i] += point[i]
    return list_to_point([sum[i]/l for i in range(3)])

#>===[ LIST TO POINT ]===<#
def list_to_point(l):
    p = Point()
    p.x = l[0]
    p.y = l[1]
    p.z = l[2]
    return p

#>===[ DISTANCE BETWEEN 2 POINTS ]===<#
def get_distance(p1, p2):
    return sum([(p2[i]-p1[i])**2 for i in range(2)])

#>===[ SUB NODE ]===<#
class GPSConverter(Node):

    # --- Initialization --- #
    def __init__(self):
        super().__init__('lidar_converter_gps_client')
        self.cli = self.create_client(ConvertToGPS, '/team52/convert_to_gps')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = ConvertToGPS.Request()

    def send_request(self, point):
        self.request.cart = point
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().gps

#>===[ NODE ]===<#
class LidarConverter(Node):

    # --- Initialization --- #
    def __init__(self):
        super().__init__("lidar_converter")

        # Attributes #
        self.tilts = [0,0,0]

        # Subscribers #
        self.subscriber_converter = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.converter,
            10
        )
        self.subscriber_orientation = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.orientation,
            10
        )

        # Publishers #
        self.obs_publisher = self.create_publisher(Obstacles, "/team52/obstacles", 10)
        self.boat_publisher = self.create_publisher(Obstacles, "/team52/boats", 10)

        # Service #
        self.client_gps_converter = GPSConverter()
        self.get_logger().info("Node ready")
    
    # --- Set tilts --- #
    def orientation(self, data: Imu):
        w = data._orientation.w
        x = data._orientation.x
        y = data._orientation.y
        z = data._orientation.z
        self.tilts = quaternion_to_tilts(w,x,y,z)

    # --- Correct the height --- #
    def vertical_correction(self, point):
        point[2] -= (point[0]*self.tilts[0]) - (point[1]*self.tilts[1])

    def converter(self, point_mat: PointCloud2):
        # > Processing points < #
        object_point = []
        # Selecting only 4 layers #
        for h in range(6,10):
            # Selecting only 1 point out of 20 #
            for w in range(0, point_mat.width, 20):
                offset = w*point_mat.point_step + h*point_mat.row_step
                point = get_point(point_mat.data, offset)
                self.vertical_correction(point)
                # If match the 'object' description -> add to the list #
                if 5 < abs(point[0]) < 600 and 5 < abs(point[1]) < 600 and point[2] > -0.6:
                    object_point.append(point)
                    object_point[-1].append([0,0])
        if len(object_point) == 0:
            obstacles_msg = Obstacles()
            self.obs_publisher.publish(obstacles_msg) 
            return

        # > Create groups < #
        for coord in range(2):
            gr_num = 0
            object_point.sort(key=lambda x: x[coord])
            for i in range(1, len(object_point)):
                if abs(object_point[i][coord]-object_point[i-1][coord]) > 10:
                    gr_num += 1
                object_point[i][3][coord] = gr_num

        # > Split groups < #
        object_point.sort(key=lambda x: x[3])
        group_list = [[object_point[0]]]
        for i in range(1, len(object_point)):
            if object_point[i][3] != group_list[-1][0][3]:
                group_list.append([])
            group_list[-1].append(object_point[i])

        # > Processing groups < #
        boats = []
        obstacles = []
        for group in group_list:
            # Get max height #
            group.sort(key=lambda x: x[2])
            # If too short -> Boat #
            if group[-1][2] < 0.6:
                boats.append(self.client_gps_converter.send_request(average_point(group)))
            # Else -> Obstacle #
            else:
                max_distance = [0,0,0]
                l = len(group)
                # Get all distances #
                for i in range(l):
                    for j in range(i+1,l):
                        distance = get_distance(group[i], group[j])
                        # Save the longest #
                        if distance > max_distance[2]:
                            max_distance = [i,j,distance]
                # Print result #
                i,j,d = max_distance
                # First point #
                response = self.client_gps_converter.send_request(list_to_point(group[i]))
                obstacles.append(response)
                # Second point #
                response = self.client_gps_converter.send_request(list_to_point(group[j]))
                obstacles.append(response)
        # > Publish boats < #
        boats_msg = Obstacles()
        boats_msg.size = len(boats)
        boats_msg.gps_list = boats
        self.boat_publisher.publish(boats_msg)
        # > Publish obstacles < #
        obstacles_msg = Obstacles()
        obstacles_msg.size = len(obstacles)//2
        obstacles_msg.gps_list = obstacles
        self.obs_publisher.publish(obstacles_msg)

def main(args=None):
    # Initiation #
    rclpy.init(args=args)
    converter = LidarConverter()
    
    # Core #
    rclpy.spin(converter)

    # Shutdown #
    rclpy.shutdown()

if __name__ == "__main__":
    main()