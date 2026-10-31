import time

import rclpy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SubscriberVelocity(Node):
    def __init__(self):
        super().__init__('subscriber_velocity')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.msg_callback,
            10
        )
        self.wheel_base = 0.20
        self.timer_interval = 0.02

        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim successfully.')

            self.robotHandle = self.sim.getObject('/myRobot')
            self.left_signal = str(self.robotHandle) + 'leftVel'
            self.right_signal = str(self.robotHandle) + 'rightVel'

            self.sim.startSimulation()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = 0.0
        self.timer = self.create_timer(self.timer_interval, self.send_velocity)

    def msg_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def send_velocity(self):
        if time.monotonic() - self.last_cmd_time > 0.4:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear
            angular = self.angular

        left_velocity = linear - angular * self.wheel_base / 2.0
        right_velocity = linear + angular * self.wheel_base / 2.0

        self.sim.setFloatSignal(self.left_signal, left_velocity)
        self.sim.setFloatSignal(self.right_signal, right_velocity)

    def stop(self):
        try:
            self.sim.setFloatSignal(self.left_signal, 0.0)
            self.sim.setFloatSignal(self.right_signal, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
