import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from team52_interfaces.msg import Obstacles
from team52_interfaces.srv import ConvertToCart

#>===[ FUNCTIONS ]===<#

# > Get distance < #
def get_distance(point_a: Point, point_b: Point):
    return math.sqrt((point_a.x-point_b.x)**2+(point_a.y-point_b.y)**2)

#>===[ SUB NODE ]===<#
class GPSConverter(Node):

    # --- Initialization --- #
    def __init__(self):
        super().__init__('lidar_converter_gps_client')
        self.cli = self.create_client(ConvertToCart, '/team52/convert_to_cart')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = ConvertToCart.Request()

    def send_request(self, ais: NavSatFix):
        self.request.gps = ais
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().cart


class Filter(Node):

    def __init__(self):
        super().__init__("allies_detection")
        self.origin = Point()

        # > Subscribers < #
        # --- AIS --- #
        self.allies_positions = []
        self.allies_distances = []
        self.ais_subscriber = self.create_subscription(
            PoseArray,
            "/wamv/ais_sensor/allies_positions",
            self.get_positions,
            10
        )
        # --- Lidar --- #
        self.lidar_subscriber = self.create_subscription(
            Obstacles,
            "/team52/obstacles",
            self.compare,
            10
        )

        # > Publishers < #
        self.lidar_enemy_publisher = self.create_publisher(
            NavSatFix,
            '/team52/lidar_enemy',
            10
        )

        # Service #
        self.client_gps_converter = GPSConverter()
        self.get_logger().info("Node ready")


    def get_positions(self, ais_list: PoseArray):
        # Get positions #
        new_positions = []
        new_distances = []
        obstacles = []
        for ais in ais_list.poses:
            gps = NavSatFix()
            gps.longitude = ais.position.y
            gps.latitude = ais.position.x
            gps.altitude = ais.position.z
            new_positions.append(gps)
            new_distances.append(self.client_gps_converter.send_request(gps))
            # Publish obstacle #
            margin = 30
            dist = get_distance(self.origin, new_distances[-1])
            angle = math.atan2(new_distances[-1].y, new_distances[-1].x)
            delta_angle = math.acos((dist-margin)/dist)
            new_point = Point()
            new_point.z = new_distances[-1].z
            new_point.x = dist*math.cos(angle+delta_angle)
            new_point.y = dist*math.sin(angle+delta_angle)
            obstacles.append(new_point)
            new_point.x = dist*math.cos(angle-delta_angle)
            new_point.y = dist*math.sin(angle-delta_angle)
            obstacles.append(new_point)
        self.allies_distances = new_distances

    def get_distance_min(self, gps: NavSatFix):
        cart = self.client_gps_converter.send_request(gps)
        distance_min = 720000
        for d in self.allies_distances:
            distance_min = min(distance_min, get_distance(d, cart))
        print(self.allies_distances)
        print(distance_min)
        return distance_min
    
    def compare(self, lidar_coord: Obstacles):
        for ais in lidar_coord.gps_list:
            if self.get_distance_min(ais) > 100:
                self.lidar_enemy_publisher.publish(ais)
                return
        

def main():
    rclpy.init()

    filter = Filter()
    rclpy.spin(filter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
