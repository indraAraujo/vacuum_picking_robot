from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

class CoppeliaTFPublisher(Node):
    def __init__(self):
        super().__init__('coppelia_tf_publisher')

        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            # Get robot handle
            self.robotHandle = self.sim.getObject('/vacuumCleaner')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            raise

        # Request 50 Hz; the actual rate depends on Coppelia remote-API latency.
        self.tf_publish_rate = self.declare_parameter("tf_publish_rate", 50.0).value
        if self.tf_publish_rate <= 0.0:
            raise ValueError("tf_publish_rate must be positive")
        self.timer = self.create_timer(1.0 / self.tf_publish_rate, self.publish_transforms)

    def get_transform(self, child_handle, child_frame, parent_handle, parent_frame):
        pos = self.sim.getObjectPosition(child_handle, parent_handle)
        quat = self.sim.getObjectQuaternion(child_handle, parent_handle)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        return t

    def publish_transforms(self):
        # Build and send multiple transforms
        # Use robot handle directly for odom -> base_link
        transforms = [
            self.get_transform(self.robotHandle, 'base_link', -1, 'odom'),
        ]

        for t in transforms:
            self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()