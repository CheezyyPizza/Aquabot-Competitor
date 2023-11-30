import rclpy
from rclpy.node import Node

import math

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class Camera(Node):

    def __init__(self):
        super().__init__('camera')

        # > Subscriber < #
        self.camera_subs = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/main_camera_sensor/image_raw',
            self.get_data,
            2
        )

        # > Publisher < #
        self.angle_pubs = self.create_publisher(
            Float32,
            'team52/enemy_angle',
            1
        )
    
    def get_data(self, data: Image):
        tolerance = 5
        rref, gref, bref = 67,8,5
        average_red = [0,0]
        max_red = 0
        for h in range(data.height):
            for w in range(data.width):
                r,g,b = data.data[h*data.step+w*3:h*data.step+(w+1)*3]
                if abs(r-rref) < tolerance and abs(g-gref) < tolerance and abs(b-bref) < tolerance:
                    average_red[0] += w
                    average_red[1] += 1
                    max_red = max(max_red, h)
                # if h != 0:
                #     ur, ug, ub = data.data[(h-1)*data.step+w*3:(h-1)*data.step+(w+1)*3]
                #     eps = 15
                #     if abs(ur-r-20) < eps and abs(ug-g-20) < eps and abs(ub-b-20) < eps:
                #         line[h//9] += 1
        fov = math.radians(80)
        angle = Float32()
        angle.data = 10.
        if average_red[1] != 0:
            ratio = fov/data.width
            angle_from_left = ratio*average_red[0]/average_red[1]
            angle.data = fov/2-angle_from_left
            print(max_red)
            if max_red >= 200:
                angle.data = -10.
        self.angle_pubs.publish(angle)

def main():
    rclpy.init()

    camera = Camera()
    rclpy.spin(camera)

    rclpy.shutdown()

if __name__ == "__main__":
    main()