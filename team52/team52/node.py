import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from team52_interfaces.msg import Obstacles

class Publisher(Node):
    def __init__(self):
        super().__init__("noeud_exemple")
        self.publisher = self.create_publisher(Obstacles, "/team52/test", 10)
        self.create_timer(0.5, self.callback)
    
    def callback(self):
        donnee1 = NavSatFix()
        donnee1.latitude = 1.
        donnee1.longitude = 2.
        donnee2 = NavSatFix()
        donnee2.latitude = 15.
        donnee2.longitude = -52.
        data = Obstacles()
        data.points.append(donnee1)
        data.points.append(donnee2)
        self.publisher.publish(data)


def main():
    rclpy.init()

    publisher = Publisher()
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
