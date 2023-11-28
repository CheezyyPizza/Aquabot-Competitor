import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray

class PathFinder(Node):

    def __init__(self):
        super().__init__("path_finder")

        # > Subscription < #
        self.obj_subs = self.create_subscription(
            PoseArray,
            "/wamv/ais_sensor/allies_positions",
            self.get_position,
            10
        )

        # > Publisher < #
        self.waypont_pubs = self.create_publisher(
            NavSatFix,
            '/team52/waypoint',
            10
        )

    def get_position(self, data):
        gps = NavSatFix()
        gps.longitude = data.poses[1].position.y
        gps.latitude = data.poses[1].position.x
        gps.altitude = data.poses[1].position.z
        self.waypont_pubs.publish(gps)


def main():
    rclpy.init()

    pathfinder = PathFinder()
    rclpy.spin(pathfinder)

    rclpy.shutdown()

if __name__ == "__main__":
    main()