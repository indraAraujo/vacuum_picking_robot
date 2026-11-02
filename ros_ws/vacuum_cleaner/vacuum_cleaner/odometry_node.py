import math
import threading
import time

import rclpy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.robot = self.sim.getObject('/myRobot')
        position = self.sim.getObjectPosition(self.robot, -1)
        orientation = self.sim.getObjectOrientation(self.robot, -1)
        self.x0, self.y0 = position[0], position[1]
        self.yaw0 = orientation[2]

        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_sim_time = self.sim.getSimulationTime()
        self.running = True

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.reader = threading.Thread(target=self.read_robot_pose, daemon=True)
        self.reader.start()

        self.timer = self.create_timer(0.02, self.publish_odometry)

        self.get_logger().info('Odometry node started')

    def read_robot_pose(self):
        while self.running and rclpy.ok():
            try:
                position = self.sim.getObjectPosition(self.robot, -1)
                orientation = self.sim.getObjectOrientation(self.robot, -1)
                sim_time = self.sim.getSimulationTime()
            except Exception as e:
                self.get_logger().error(f'Failed to read CoppeliaSim state: {e}')
                time.sleep(0.5)
                continue

            with self.lock:
                dx = position[0] - self.x0
                dy = position[1] - self.y0
                c = math.cos(self.yaw0)
                s = math.sin(self.yaw0)
                x = c * dx + s * dy
                y = -s * dx + c * dy
                theta = math.atan2(
                    math.sin(orientation[2] - self.yaw0),
                    math.cos(orientation[2] - self.yaw0))

                dt = sim_time - self.last_sim_time
                if dt > 0.0:
                    dtheta = math.atan2(
                        math.sin(theta - self.theta),
                        math.cos(theta - self.theta))
                    self.v = (
                        (x - self.x) * math.cos(self.theta)
                        + (y - self.y) * math.sin(self.theta)) / dt
                    self.w = dtheta / dt
                else:
                    self.v = 0.0
                    self.w = 0.0

                self.x = x
                self.y = y
                self.theta = theta
                self.last_sim_time = sim_time

            time.sleep(0.1)

    def publish_odometry(self):
        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
            v = self.v
            w = self.w

        current_time = self.get_clock().now()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
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
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def stop(self):
        self.running = False
        if self.reader.is_alive():
            self.reader.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
