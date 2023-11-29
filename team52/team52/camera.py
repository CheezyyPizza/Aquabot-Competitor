import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

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
    
    def get_data(self, data: Image):
        print(">=====[ NEW DATA ]=====<")
        tolerance = 10
        rref, gref, bref = 67,8,5
        mat = []
        lowest_red = [0,0]
        line = [0]*64
        for i in range(64):
            mat.append(['.']*80)
        for h in range(data.height):
            for w in range(data.width):
                r,g,b = data.data[h*data.step+w*3:h*data.step+(w+1)*3]
                if abs(r-rref) < tolerance and abs(g-gref) < tolerance and abs(b-bref) < tolerance and h > lowest_red[0]:
                    lowest_red = [h,w]
                if h != 0:
                    ur, ug, ub = data.data[(h-1)*data.step+w*3:(h-1)*data.step+(w+1)*3]
                    eps = 15
                    if abs(ur-r-20) < eps and abs(ug-g-20) < eps and abs(ub-b-20) < eps:
                        line[h//9] += 1
        if lowest_red[0] != 0:
            mat[lowest_red[0]//9][lowest_red[1]//9] = '#'
        max_line = max(line)
        for i in range(64):
            if line[i] == max_line:
                mat[i] = ['_' if mat[i][w] == '.' else '#' for w in range(80)]
            print(''.join(mat[i]))

def main():
    rclpy.init()

    camera = Camera()
    rclpy.spin(camera)

    rclpy.shutdown()

if __name__ == "__main__":
    main()