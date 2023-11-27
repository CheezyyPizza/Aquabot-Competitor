import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from team52_interfaces.msg import Obstacles
from team52_interfaces.srv import ConvertToCart
from team52_interfaces.srv import BeaconToGPS

#>===[ FUNCTIONS ]===<#

# > Get distance < #
def get_distance(point_a: Point, point_b: Point):
    return math.sqrt((point_a.x-point_b.x)**2+(point_a.y-point_b.y)**2)

#>===[ SUB NODE ]===<#
class GPSConverter(Node):

    # --- Initialization --- #
    def __init__(self):
        super().__init__('lidar_converter_gps_client')
        # To cart #
        self.cli_cart = self.create_client(ConvertToCart, '/team52/convert_to_cart')
        while not self.cli_cart.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request_gps = ConvertToCart.Request()
        # Beacon To GPS #
        self.cli_btogps = self.create_client(BeaconToGPS, '/team52/beacon_to_gps')
        while not self.cli_btogps.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request_beacon = BeaconToGPS.Request()

    def send_request_tocart(self, ais: NavSatFix):
        self.request_gps.gps = ais
        self.future_cart = self.cli_cart.call_async(self.request_gps)
        rclpy.spin_until_future_complete(self, self.future_cart)
        return self.future_cart.result().cart

    def send_request_beacontogps(self, distance, angle):
        self.request_beacon.distance = distance
        self.request_beacon.angle = angle
        self.future_gps = self.cli_btogps.call_async(self.request_beacon)
        rclpy.spin_until_future_complete(self, self.future_gps)
        return self.future_gps.result().gps

class Filter(Node):

    def __init__(self):
        super().__init__("allies_detection")
        self.origin = Point()
        self.beacon_ais = Point()

        # > Subscribers < #
        # --- AIS --- #
        self.allies_cart = []
        self.ais_subscriber = self.create_subscription(
            PoseArray,
            "/wamv/ais_sensor/allies_positions",
            self.get_positions,
            10
        )
        self.beacon_subscriber = self.create_subscription(
            NavSatFix,
            "/team52/beacon",
            self.get_beacon,
            10
        )
        # --- Lidar --- #
        self.lidar_subscriber = self.create_subscription(
            Obstacles,
            "/team52/boats",
            self.compare,
            10
        )

        # > Publishers < #
        self.lidar_enemy_publisher = self.create_publisher(
            NavSatFix,
            '/team52/lidar_enemy',
            10
        )
        self.boat_obs_publisher = self.create_publisher(
            Obstacles,
            '/team52/boat_obstacles',
            10
        )

        # Service #
        self.client_gps_converter = GPSConverter()
        self.get_logger().info("Node ready")

    def get_beacon(self, gps: NavSatFix):
        self.beacon_ais = gps

    def get_positions(self, ais_list: PoseArray):
        # Get positions #
        new_ais = []
        obstacles = []
        for ais in ais_list.poses:
            gps = NavSatFix()
            gps.longitude = ais.position.y
            gps.latitude = ais.position.x
            gps.altitude = ais.position.z
            new_ais.append(gps)
            # Set obstacles #
            cart = self.client_gps_converter.send_request_tocart(gps)
            margin = 10
            dist = get_distance(self.origin, cart)
            angle = math.atan2(cart.y, cart.x)
            delta_angle = 2*math.asin(margin/(2*dist))
            # Set new points #
            obstacles.append(self.client_gps_converter.send_request_beacontogps(dist, angle+delta_angle))
            obstacles.append(self.client_gps_converter.send_request_beacontogps(dist, angle-delta_angle))
        # > Beacon < #
        new_ais.append(self.beacon_ais)
        # Set obstacles #
        try:
            cart = self.client_gps_converter.send_request_tocart(self.beacon_ais)
            margin = 5
            dist = get_distance(self.origin, cart)
            angle = math.atan2(cart.y, cart.x)
            delta_angle = 2*math.asin(margin/(2*dist))
            # Set new points #
            obstacles.append(self.client_gps_converter.send_request_beacontogps(dist, angle+delta_angle))
            obstacles.append(self.client_gps_converter.send_request_beacontogps(dist, angle-delta_angle))
        except:
            pass
        # > Process results < #
        self.allies_ais = new_ais
        obstacles_msg = Obstacles()
        obstacles_msg.size = len(obstacles)//2
        obstacles_msg.gps_list = obstacles
        self.boat_obs_publisher.publish(obstacles_msg)

    def get_distance_min(self, gps: NavSatFix):
        cart = self.client_gps_converter.send_request_tocart(gps)
        distance_min = 600
        for ally_ais in self.allies_ais:
            ally_cart = self.client_gps_converter.send_request_tocart(ally_ais)
            distance_min = min(distance_min, get_distance(ally_cart, cart))
        return distance_min
    
    def compare(self, lidar_coord: Obstacles):
        for ais in lidar_coord.gps_list:
            distance = self.get_distance_min(ais)
            if distance > 50:
                print(distance, self.client_gps_converter.send_request_tocart(ais))
                self.lidar_enemy_publisher.publish(ais)
                return
        
        

def main():
    rclpy.init()

    filter = Filter()
    rclpy.spin(filter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
