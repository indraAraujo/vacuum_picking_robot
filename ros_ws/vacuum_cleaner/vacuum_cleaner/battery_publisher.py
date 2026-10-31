import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

CHARGING_SIGNAL_ON = 1
CHARGING_SIGNAL_OFF = 0


class BatteryInformationNode(Node):
    def __init__(self):
        super().__init__("battery_information_node")

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.robotHandle = self.sim.getObject('/myRobot')

        self.msg_topic = self.create_publisher(BatteryState, 'myRobot/battery', 10)

        self.timer = self.create_timer(0.1, self.publish_battery_information)

    def publish_battery_information(self):
        battery = self.sim.getFloatSignal(str(self.robotHandle) + "Battery")
        charging = self.sim.getInt32Signal(str(self.robotHandle) + "Charging")

        if not isinstance(battery, float):
            return

        battery_msg = BatteryState()
        battery_msg.percentage = battery
        battery_msg.present = True
        if charging == CHARGING_SIGNAL_ON:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif charging == CHARGING_SIGNAL_OFF:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        self.msg_topic.publish(battery_msg)


def main(args=None):
    rclpy.init(args=args)
    battery_information_node = BatteryInformationNode()
    rclpy.spin(battery_information_node)
    battery_information_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

