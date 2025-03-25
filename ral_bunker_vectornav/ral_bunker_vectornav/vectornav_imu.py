import rclpy
import vnpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu


import tf_transformations

class Vectornav(Node):

    def __init__(self):
        super().__init__('vectornav')
        self.vs = vnpy.VnSensor()
        self.cd = vnpy.CompositeData()
        self.vs.connect("/dev/ttyUSB0", 115200)

        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        timer_period = 0.03 # In Hz: 30 Hz approx.
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        yaw, pitch, roll = self.vs.read_yaw_pitch_roll()
        imu_measurements = self.vs.read_imu_measurements()

        accel = imu_measurements.accel
        gyro = imu_measurements.gyro

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        msg.angular_velocity.x = gyro.x
        msg.angular_velocity.y = gyro.y
        msg.angular_velocity.z = gyro.z

        msg.linear_acceleration.x = accel.x
        msg.linear_acceleration.y = accel.y
        msg.linear_acceleration.z = accel.z

        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    vectornav = Vectornav()
    rclpy.spin(vectornav)
    vectornav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()