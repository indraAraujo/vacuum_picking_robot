import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.robot_handle = self.sim.getObject('/vacuumCleaner')
        self.left_motor = self.sim.getObject('/vacuumCleaner/leftMotor')
        self.right_motor = self.sim.getObject('/vacuumCleaner/rightMotor')

        self.wheel_radius = 0.0975
        self.wheel_base = 0.381

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info('Odometry node started')

    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        left_vel = self.sim.getJointVelocity(self.left_motor)
        right_vel = self.sim.getJointVelocity(self.right_motor)

        v = (left_vel + right_vel) * self.wheel_radius / 2.0
        w = (right_vel - left_vel) * self.wheel_radius / self.wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = w

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()