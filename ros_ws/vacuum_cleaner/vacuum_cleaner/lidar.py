import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

class PublisherLaserscan(Node):
    def __init__(self):
        super().__init__('laserscan_bridge')

        self.declare_parameter('scanning_density', 2.5)
        density = float(self.get_parameter('scanning_density').value)

        self.laserscan_publisher = self.create_publisher(LaserScan,'/scan', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')

            density = min(5.0, max(0.1, density))
            laser_handle = self.sim.getObject('/myRobot/LaserScanner_2D')
            laser_script = self.sim.getScript(
                self.sim.scripttype_childscript, laser_handle)
            self.sim.setScriptSimulationParameter(
                laser_script, 'scanningDensity', density)
            self.scanning_density = density

            self.get_logger().info(
                f'Connected to CoppeliaSim; scanningDensity={density}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        self.timer_period_sec = 0.1

        self.laserscan_msg = LaserScan()
        self.laserscan_msg.header.frame_id = "laser_link"
        self.laserscan_msg.angle_increment = math.pi / (density * 180.0)
        self.laserscan_msg.angle_min = -90 * (math.pi/180)
        self.laserscan_msg.angle_max = (90* (math.pi/180)) - self.laserscan_msg.angle_increment
        self.laserscan_msg.time_increment = 0.0
        self.laserscan_msg.scan_time = self.timer_period_sec
        self.laserscan_msg.range_min = 0.055
        self.laserscan_msg.range_max = 10.0

        self.timer = self.create_timer(self.timer_period_sec, self.publish_laserscan)
        
        self.get_logger().info('ROS 2 → CoppeliaSim Laserscan bridge started.')

    def publish_laserscan(self):
        distance = []
        distance = self.sim.getFloatArrayProperty(self.robotHandle, "signal.distSignal", {'noError' : True})
        
        if not distance:
            return

        now = self.get_clock().now()
        self.laserscan_msg.header.stamp = now.to_msg()
        self.laserscan_msg.ranges = distance
        self.laserscan_msg.angle_increment = math.pi / (self.scanning_density * 180.0)
        self.laserscan_msg.angle_max = (
            self.laserscan_msg.angle_min
            + (len(distance) - 1) * self.laserscan_msg.angle_increment)

        self.laserscan_publisher.publish(self.laserscan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherLaserscan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
