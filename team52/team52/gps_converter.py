import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from team52_interfaces.srv import ConvertToGPS
from team52_interfaces.srv import BeaconToGPS
from team52_interfaces.srv import ConvertToCart

class Coordonnees(Node):

    def __init__(self):
        super().__init__('coorodnnees')

        self.longitude = 0
        self.latitude = 0
        self.altitude = 0
        self.yaw = 0
        self.subs = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.set_coord, 10)
        self.srv_cg = self.create_service(ConvertToGPS, '/team52/convert_to_gps', self.deplacer_coordonnees)
        self.srv_bg = self.create_service(BeaconToGPS, '/team52/beacon_to_gps', self.deplacer_coordonnees_angle)
        self.srv_gc = self.create_service(ConvertToCart, '/team52/convert_to_cart', self.calculer_ecart)

    def set_coord(self, data: NavSatFix):
        self.longitude = data.longitude
        self.latitude = data.latitude
        self.altitude = data.altitude

    def cart_to_gps(self, cart: Point):
        RT = 6371000
        x_rad = cart.x / RT
        y_rad = cart.y / RT
        gps = NavSatFix()
        gps.latitude = self.latitude + math.degrees(y_rad)
        gps.longitude = self.longitude + math.degrees(x_rad / math.cos(math.radians(self.latitude)))
        gps.altitude = self.altitude + cart.z
        return gps

    def deplacer_coordonnees(self, request, response):
        response.gps = self.cart_to_gps(request.cart)
        return response

    def deplacer_coordonnees_angle(self, request, response):
        cart = Point()
        angle_rotation = request.angle
        cart.x = request.distance * math.cos(angle_rotation)
        cart.y = request.distance * math.sin(angle_rotation)
        response.gps = self.cart_to_gps(cart)
        return response
    
    def calculer_ecart(self, request, response):
        RT = 6371000
        x_rad = math.radians(request.gps.longitude - self.longitude)*math.cos(math.radians(self.latitude))
        y_rad = math.radians(request.gps.latitude - self.latitude)
        response.cart.x = x_rad*RT
        response.cart.y = y_rad*RT
        response.cart.z = request.gps.altitude - self.altitude
        return response

def main() :
    rclpy.init()
    coordonnees = Coordonnees()

    rclpy.spin(coordonnees)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
