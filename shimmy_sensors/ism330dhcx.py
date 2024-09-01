import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import board
import busio
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .utils.mahony import Mahony

class ISM330DHCXService(Node):

    def __init__(self):
        super().__init__('ISM330DHCX_service')
        self.i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.filter = Mahony()

        self.timer_imu  = self.create_timer(1/100, self.callback)

        
        self.i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.imu = ISM330DHCX(self.i2c)

        frame_id = self.declare_parameter('imu_frame_id', "shimmy_imu_link").value
        self.accels_bias = self.declare_parameter('accels_bias', [0.,0.,0.])
        self.gyros_bias = self.declare_parameter('gyros_bias', [0.,0.,0.])
        self.declare_parameter('base_link_frame_id', 'base_link')  # Frame ID of the base link
        self.declare_parameter('translation', [0.0, 0.0, 0.0])  # Translation (x, y, z)
        self.declare_parameter('rotation', [0.0, 0.0, 0.0, 1.0])  # Rotation (quaternion: x, y, z, w)

        # Get parameter values
        imu_frame_id = self.get_parameter('imu_frame_id').value
        base_link_frame_id = self.get_parameter('base_link_frame_id').value
        translation = self.get_parameter('translation').value
        rotation = self.get_parameter('rotation').value

        # Create a static transform broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)
        # Create the transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_link_frame_id
        transform.child_frame_id = imu_frame_id
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        # Send the transform
        self.broadcaster.sendTransform(transform)
        self.get_logger().info('Published transform from {} to {}'.format(
            base_link_frame_id, imu_frame_id))

        self.pub_imu = self.create_publisher(Imu, '/shimmy_bot/imu', 100)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id

        self.imu_msg.linear_acceleration_covariance = [0.0]*9
        self.imu_msg.orientation_covariance = [0.0]*9
        self.imu_msg.angular_velocity_covariance = [0.0]*9

    def callback(self):
            a = self.imu.acceleration
            g = self.imu.gyro

            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.linear_acceleration.x = a[0]
            self.imu_msg.linear_acceleration.y = a[1]
            self.imu_msg.linear_acceleration.z = a[2]
            self.imu_msg.angular_velocity.x = g[0]
            self.imu_msg.angular_velocity.y = g[1]
            self.imu_msg.angular_velocity.z = g[2]

            # generally I would also have the magnetometer to determine heading
            # not sure I like this solution/hack :P
            # q = self.compass.get_quaternion(a, None)
            # q = Quaternion()
            q = self.filter.update(a,g,0.01)

            # ROS is JPL and mine is Hamilton quaternion
            # qJPL = qHamilton.conjugate
            self.imu_msg.orientation.x = -q.x
            self.imu_msg.orientation.y = -q.y
            self.imu_msg.orientation.z = -q.z
            self.imu_msg.orientation.w = q.w

            self.pub_imu.publish(self.imu_msg)
        

def main():
    rclpy.init()

    m_service = ISM330DHCXService()
    executor =  SingleThreadedExecutor()



    executor.add_node(m_service)

    executor.spin()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()