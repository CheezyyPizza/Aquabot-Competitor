import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from team52_interfaces.msg import Obstacles  


class Filter(Node):

    def __init__(self):
        super().__init__("allies_detection")

        # > Subscribers < #
        # --- AIS --- #
        self.allies_positions = []
        self.ais_subscriber = self.create_subscription(
            NavSatFix,
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
            Obstacles,
            '/team52/lidar_enemy',
            10
        )

    def get_positions(self, ais_list: PoseArray):
        new_positions = []
        for ais in ais_list:
            coord = NavSatFix()
            coord.longitude = ais.position.x
            coord.latitude = ais.position.y
            coord.altitude = ais.position.z
            new_positions.append(coord)
        self.allies_positions = new_positions

    def is_in_circle(self, coord: Point, range):
        pass
    
    def compare(self, lidar_coord: Obstacles):
        pass
        

def main():
    rclpy.init()

    filter = Filter()
    rclpy.spin(filter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
