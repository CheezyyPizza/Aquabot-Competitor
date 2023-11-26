import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from team52_interfaces.msg import Obstacles
from team52_interfaces.srv import ConvertToCart


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
        self.request.coord_gps = ais
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().coord_cart


class Filter(Node):

    def __init__(self):
        super().__init__("allies_detection")

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
        for ais in ais_list.poses:
            coord = NavSatFix()
            coord.longitude = ais.position.y
            coord.latitude = ais.position.x
            coord.altitude = ais.position.z
            new_positions.append(coord)
            new_distances.append(self.client_gps_converter.send_request(coord))
        self.allies_positions = new_positions
        self.allies_distances = new_distances
        # Publish obstacle #


    def get_distance_min(self, coord: NavSatFix):
        coord_cart = self.client_gps_converter.send_request(coord)
        distance_min = 720000
        for d in self.allies_distances:
            distance_min = min(distance_min, abs(coord_cart.x-d.x)**2+abs(coord_cart.y-d.y)**2)
        print(self.allies_distances)
        print(distance_min)
        return distance_min
    
    def compare(self, lidar_coord: Obstacles):
        for ais in lidar_coord.gps:
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
