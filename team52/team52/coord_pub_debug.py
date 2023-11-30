## ----------- code uniquement là pour me permettre de tester le pid -----------

import rclpy
from rclpy.node import Node

from math import atan2

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


# pos initiale du bateau: -4.9763157221789207    48.04631299831134

class CoordPublisher(Node):

    def __init__(self):
        super().__init__('coord_publisher')

        self.msg = NavSatFix()
        self.msg.longitude = -4.985
        self.msg.latitude = 48.04631299831134

        self.publisher = self.create_publisher(NavSatFix, '/team52/goal', 10)
        wait_for_change = 30
        self.change_time = self.create_timer(wait_for_change, self.change_callback)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.run()

    def change_callback(self):
        self.msg.longitude = -4.970
        self.msg.latitude = 48.04631299831134
        self.get_logger().info("changed coord")

    def timer_callback(self):
        self.publisher.publish(self.msg)
        self.i += 1

    def run(self):
        self.get_logger().info("running")
        while rclpy.ok():

            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    coord_publisher = CoordPublisher()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    coord_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()