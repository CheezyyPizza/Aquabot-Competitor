import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray

class PathFinder(Node):

    def __init__(self):
        super().__init__("path_finder")

        # > Subscription < #
        self.obj_subs = self.create_subscription(
            NavSatFix,
            "/team52/objective",
            self.get_position,
            10
        )

        # > Publisher < #
        self.waypont_pubs = self.create_publisher(
            NavSatFix,
            '/team52/waypoint',
            10
        )

    def get_position(self, gps):
        self.waypont_pubs.publish(gps)


def main():
    rclpy.init()

    pathfinder = PathFinder()
    rclpy.spin(pathfinder)

    rclpy.shutdown()

if __name__ == "__main__":
    main()