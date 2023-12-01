import rclpy
from rclpy.node import Node

import math

from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from team52_interfaces.srv import ConvertToCart
from team52_interfaces.srv import BeaconToGPS

#>===[ QUATERNION TO YAW ]===<#
def quaternion_to_yaw(qw, qx, qy, qz):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

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

class EnemyDectector(Node):
    
    def __init__(self):
        super().__init__('enemy_detector')

        # > Subscribers < #
        self.angle_subs = self.create_subscription(
            Float32,
            '/team52/enemy_angle',
            self.get_angle,
            1
        )

        self.lidar_subs = self.create_subscription(
            NavSatFix,
            '/team52/enemy_lidar',
            self.get_lidar,
            1
        )
        self.enemy_lidar = NavSatFix()

        self.yaw = 0
        self.subs_tilts = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.orientation,
            5
        )

        # > Publisher < #
        self.enemy_pub = self.create_publisher(
            NavSatFix,
            '/team52/enemy',
            1
        )

        # > Client < #
        self.gps_converter = GPSConverter()
        self.get_logger().info("Node ready")

    # --- Set tilts --- #
    def orientation(self, data: Imu):
        w = data._orientation.w
        x = data._orientation.x
        y = data._orientation.y
        z = data._orientation.z
        self.yaw = quaternion_to_yaw(w,x,y,z)

    def get_angle(self, data):
        # Gather information #
        angle = data.data + self.yaw
        fov = math.radians(80)

        # Process #
        enemy_cart = self.gps_converter.send_request_tocart(self.enemy_lidar)
        angle_lidar = math.atan2(enemy_cart.y, enemy_cart.x)

        result = NavSatFix()

        if angle > 5.:
            if self.enemy_lidar.longitude != 0:
                if abs(angle_lidar-self.yaw) < fov/2:
                    print("No boat")
                    self.enemy_lidar = NavSatFix()
                else:
                    print("Blind boat")
                    result = self.enemy_lidar
        else:
            camera_gps = NavSatFix()
            if angle > -5.:
                camera_gps = self.gps_converter.send_request_beacontogps(50., angle)
                print(angle)
            elif angle < -50.:
                camera_gps = self.gps_converter.send_request_beacontogps(5., angle)
                print("ALERT !")
            else:
                print("too close")
            if self.enemy_lidar.longitude == 0:
                result = camera_gps
                print("Only cam")
            else:
                print(angle_lidar, self.yaw, min(abs(angle_lidar-self.yaw),2*math.pi-abs(angle_lidar-self.yaw)))
                if min(abs(angle_lidar-angle),2*math.pi-abs(angle_lidar-angle)) < math.radians(5):
                    result = self.enemy_lidar
                    print("Cam + lidar")
                else:
                    self.enemy_lidar = NavSatFix()
                    result = camera_gps
                    print("Cam")
        
        cart = self.gps_converter.send_request_tocart(result)
        print("X:{0:4f} Y:{1:4f}\n\tE:{2:8f} N:{3:8f}".format(cart.x, cart.y, result.longitude, result.latitude))
        self.enemy_pub.publish(result)

            
    def get_lidar(self, gps):
        self.enemy_lidar = gps

def main():
    rclpy.init()

    enemy_detector = EnemyDectector()
    rclpy.spin(enemy_detector)

    rclpy.shutdown

if __name__ == '__main__':
    main()