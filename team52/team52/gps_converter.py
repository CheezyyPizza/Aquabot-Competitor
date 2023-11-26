import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from team52_interfaces.srv import ConvertToGPS
from team52_interfaces.srv import ConvertToCart

class Coordonnees(Node):

    def __init__(self):
        super().__init__('coorodnnees')

        self.longitude = 0
        self.latitude = 0
        self.altitude = 0
        self.subs = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.set_coord, 10)

        self.srv = self.create_service(ConvertToGPS, '/team52/convert_to_gps', self.deplacer_coordonnees)
        self.srv = self.create_service(ConvertToCart, '/team52/convert_to_cart', self.calculer_ecart)
        self.get_logger().info("Node ready")

    def set_coord(self, data: NavSatFix):
        self.longitude = data.longitude
        self.latitude = data.latitude
        self.altitude = data.altitude

    def deplacer_coordonnees(self, request, response):
        RT = 6371000
        x_rad = request.coord_cart.x / RT
        y_rad = request.coord_cart.y / RT
        response.coord_gps.latitude = self.latitude + math.degrees(y_rad)
        response.coord_gps.longitude = self.longitude + math.degrees(x_rad / math.cos(math.radians(self.latitude)))
        response.coord_gps.altitude = self.altitude + request.coord_cart.z
        
        return response

    def calculer_ecart(self, request, response):
        RT = 6371000
        x_rad = math.radians(request.coord_gps.longitude - self.longitude)*math.cos(math.radians(self.latitude))
        y_rad = math.radians(request.coord_gps.latitude - self.latitude)
        response.coord_cart.x = x_rad*RT
        response.coord_cart.y = y_rad*RT
        response.coord_cart.z = request.coord_gps.altitude - self.altitude
        return response

def main() :
    rclpy.init()
    coordonnees = Coordonnees()

    rclpy.spin(coordonnees)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()