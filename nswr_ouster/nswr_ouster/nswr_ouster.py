import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from std_msgs.msg import String
import math
import numpy as np
import cv2

class Ouster(Node):

    def __init__(self):
        super().__init__('nswr_ouster')

        # Subcribe data from the scanner
        self.subscription = self.create_subscription(PointCloud2,'/os_cloud_node/points',self.ouster_callback,1)

        # Fields in PointCloud2 ROS message for velodyne
        self.ouster_fields = [
            PointField(name='x',
                       offset=0,
                       datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='y',
                       offset=4,
                       datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='z',
                       offset=8,
                       datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='intensity',
                       offset=12,
                       datatype=PointField.UINT32,
                       count=1),
        ]

    def ouster_callback(self, msg):
        points = pc2.read_points_list(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)

        SCANNER_COLS = 1024
        SCANNER_ROWS = 128
        MAX_V_ANGLE = 22.5
        MIN_V_ANGLE = -22.5
        V_ANGLE_RANGE = MAX_V_ANGLE - MIN_V_ANGLE

        intensity_img = np.zeros((SCANNER_ROWS, SCANNER_COLS), dtype=np.uint8)

        for point in points:
            # Calculating vertical and horizontal angles
            distance = math.sqrt(point.x ** 2 + point.y ** 2)
            vert_angle = math.atan2(point.z, distance) * (180 / math.pi)  # Convert to degrees
            horiz_angle = math.atan2(point.y, point.x) * (180 / math.pi)  # Convert to degrees

            if vert_angle < MIN_V_ANGLE or vert_angle > MAX_V_ANGLE:
                continue

            vert_pixel = int((vert_angle - MIN_V_ANGLE) / V_ANGLE_RANGE * (SCANNER_ROWS - 1))
            horiz_pixel = int((horiz_angle + 180) / 360 * (SCANNER_COLS - 1))

            # Map intensity to pixel value
            intensity_value = int(point.intensity)
            if intensity_value >= 255:
                intensity_value = 255

            intensity_img[vert_pixel, horiz_pixel] = intensity_value
        
        intensity_img=cv2.flip(intensity_img,0)

        # Show image using OpenCV
        cv2.imshow("Intensity Image", intensity_img)
        cv2.waitKey(1)

        cv2.imwrite("intensity_image.png", intensity_img)



def main(args=None):
    
    rclpy.init(args=args)
    ouster = Ouster()

    rclpy.spin(ouster)

    ouster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
