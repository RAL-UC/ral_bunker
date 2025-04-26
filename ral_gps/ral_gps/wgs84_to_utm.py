import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
import utm

class WGS84ToUTM(Node):
    def __init__(self):
        super().__init__('wgs84_to_utm_node')

        self.subscription_gps = self.create_subscription( NavSatFix, '/fix', self.gps_callback, 10)

        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.publisher = self.create_publisher(PoseStamped, '/fix_utm', 10)

        self.latest_orientation = None
        self.get_logger().info('GPS to UTM with IMU node has been started!')

    def imu_callback(self, msg: Imu):
        self.latest_orientation = msg.orientation  # Save latest orientation

    def gps_callback(self, msg: NavSatFix):
        try:
            easting, northing, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        except Exception as e:
            self.get_logger().error(f'Failed to convert lat/lon to UTM: {e}')
            return

        utm_pose = PoseStamped()
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.header.frame_id = f"utm_{zone_number}{zone_letter}"

        utm_pose.pose.position.x = easting
        utm_pose.pose.position.y = northing
        utm_pose.pose.position.z = msg.altitude

        if self.latest_orientation:
            utm_pose.pose.orientation = self.latest_orientation
        else:
            utm_pose.pose.orientation.x = 0.0
            utm_pose.pose.orientation.y = 0.0
            utm_pose.pose.orientation.z = 0.0
            utm_pose.pose.orientation.w = 1.0
            self.get_logger().warn('No IMU data yet; publishing default orientation.')

        self.publisher.publish(utm_pose)

def main(args=None):
    rclpy.init(args=args)
    node = WGS84ToUTM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
