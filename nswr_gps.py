import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
import utm

class GPS(Node):

    def __init__(self):
        super().__init__('nswr_gps')
        self.subscription_tapas = self.create_subscription(
            NavSatFix,
            'gps_tapas',
            self.tapas_callback,
            10
        )
        
        self.subscription_magellan = self.create_subscription(
            NavSatFix,
            'dgps_magellan',
            self.magellan_callback,
            10
        )
        
        # Create topic for odometry publishers
        self.publisher_tapas_odom = self.create_publisher(Odometry, 'tapas_odom', 10)
        self.publisher_magellan_odom = self.create_publisher(Odometry, 'magellan_odom', 10)

        self.easting_offset_tapas = 0
        self.northing_offset_tapas = 0
        self.alt_offset_tapas = 0

        self.easting_offset_magellan = 0
        self.northing_offset_magellan = 0
        self.alt_offset_magellan = 0

        self.tapas_initialized = False
        self.magellan_initialized = False

        # Initialize empty file
        with open("tapas.txt", 'w') as file:
            file.close()

    def tapas_callback(self, fix):

        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return

        # Convert latitude and longitude to UTM coordinates
        u = utm.from_latlon(fix.latitude, fix.longitude)

        if not self.tapas_initialized:
            self.easting_offset_tapas = u[0]
            self.northing_offset_tapas = u[1]
            self.alt_offset_tapas = fix.altitude
            self.tapas_initialized = True

        odom = Odometry()

      
        odom.header.stamp = fix.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = fix.header.frame_id
        odom.pose.pose.position.x = u[0] - self.easting_offset_tapas
        odom.pose.pose.position.y = u[1] - self.northing_offset_tapas
        odom.pose.pose.position.z = fix.altitude - self.alt_offset_tapas

        self.publisher_tapas_odom.publish(odom)

    def magellan_callback(self, fix):

        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        u = utm.from_latlon(fix.latitude, fix.longitude)

        if not self.magellan_initialized:
            self.easting_offset_magellan = u[0]
            self.northing_offset_magellan = u[1]
            self.alt_offset_magellan = fix.altitude
            self.magellan_initialized = True

        # Declare odometry message
        odom = Odometry()

        # Fill odometry message time and frames
        odom.header.stamp = fix.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = fix.header.frame_id

        # Fill "position" in odom message
        odom.pose.pose.position.x = u[0] - self.easting_offset_magellan
        odom.pose.pose.position.y = u[1] - self.northing_offset_magellan
        odom.pose.pose.position.z = fix.altitude - self.alt_offset_magellan
        self.publisher_magellan_odom.publish(odom)

def main(args=None):

    rclpy.init(args=args)
    gps = GPS()

    rclpy.spin(gps)

    gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

