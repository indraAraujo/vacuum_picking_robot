#!/usr/bin/env python3
import math, random
import threading
import time

import rclpy
import tf2_ros
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from .modes import Mode
from coppeliasim_zmqremoteapi_client import *

WHEEL_BASE = 0.20

class MyRobot(LifecycleNode):
    def __init__(self):
        super().__init__('myrobot')
        
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self._battery = 1.0
        self._near_base = False
        self._last_cmd = Twist()
        self._mode = Mode.IDLE
        self._timer = None
        self.pub_strength = None
        self.pub_angle = None
        self.pub_batt = None
        self.sub_cmd = None
        self.sub_mode = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

        self.robotHandle = self.sim.getObject('/myRobot')

        self.left_signal = str(self.robotHandle) + 'leftVel'
        self.right_signal = str(self.robotHandle) + 'rightVel'
        self.last_cmd_time = 0.0

        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        position = self.sim.getObjectPosition(self.robotHandle, -1)
        orientation = self.sim.getObjectOrientation(self.robotHandle, -1)
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

        self.odom_publisher = self.create_publisher(Odometry, '/myRobot/odom', 10)

        self.reader = threading.Thread(target=self._on_robot_pose, daemon=True)
        self.reader.start()

        self.declare_parameter('scanning_density', 2.5)
        self.density=0.0
        self.density = min(5.0, max(0.1, self.density))
        laser_handle = self.sim.getObject('/myRobot/LaserScanner_2D')
        laser_script = self.sim.getScript(
            self.sim.scripttype_childscript, laser_handle)
        self.sim.setScriptSimulationParameter(
            laser_script, 'scanningDensity', self.density)
        self.scanning_density = self.density
        self.laserscan_publisher = self.create_publisher(LaserScan,'/scan', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ---------------- Lifecycle Callbacks ----------------
    def on_configure(self, state: State):
        self.get_logger().info('myrobot configured')
        qos = QoSProfile(depth=1)
        self.pub_strength = self.create_publisher(Float32, '/myRobot/charging_base/strengthSignal', qos)
        self.pub_angle = self.create_publisher(Float32, '/myRobot/charging_base/relativeAngle', qos)
        self.pub_batt = self.create_publisher(BatteryState, '/myRobot/battery_state', qos)
        self.sub_cmd = self.create_subscription(Twist, '/myRobot/cmd_vel', self._on_cmd, qos)
        # Operational mode subscription (2nd state)
        self.sub_mode = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        # Sim loop
        self._timer = self.create_timer(0.2, self._on_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        if self._timer is not None and self._timer.is_canceled():
            self._timer.reset()
        self.get_logger().info('myrobot activated')
        return TransitionCallbackReturn.SUCCESS

    def _cancel_timer(self):
        if self._timer is not None and not self._timer.is_canceled():
            self._timer.cancel()

    def _destroy_resources(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        for attr in ('pub_strength', 'pub_angle', 'pub_batt'):
            pub = getattr(self, attr)
            if pub is not None:
                self.destroy_publisher(pub)
                setattr(self, attr, None)
        for attr in ('sub_cmd', 'sub_mode'):
            sub = getattr(self, attr)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, attr, None)

    def on_deactivate(self, state: State):
        self.get_logger().info('myrobot deactivating: cancelling timers')
        self._cancel_timer()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info('myrobot cleaning up resources')
        self._cancel_timer()
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info('myrobot shutting down')
        self._cancel_timer()
        self._destroy_resources()
        self._shutdown_event.set()
        self._shutdown_deadline = time.monotonic() + self._shutdown_grace
        return TransitionCallbackReturn.SUCCESS

    def should_exit(self):
        if not self._shutdown_event.is_set():
            return False
        if self._shutdown_deadline is None:
            return False
        return time.monotonic() >= self._shutdown_deadline

    # ---------------- Operational Logic ----------------
    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE

    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg 
        sub_linear = msg.linear.x
        sub_angular = msg.angular.z
        if time.monotonic() - self.last_cmd_time > 0.4:
            linear = 0.0
            angular = 0.0
        else:
            linear = sub_linear
            angular = sub_angular

        left_velocity = linear - angular * WHEEL_BASE / 2.0
        right_velocity = linear + angular * WHEEL_BASE / 2.0

        self.sim.setFloatSignal(self.left_signal, left_velocity)
        self.sim.setFloatSignal(self.right_signal, right_velocity)

    def _on_transform(self, child_handle, child_frame, parent_handle, parent_frame):
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

    def _on_publish_transform(self):
        transforms = [
            self.get_transform(self.robotHandle, 'base_link', -1, 'odom'),
        ]

        for t in transforms:
            self.broadcaster.sendTransform(t)

    def _on_robot_pose(self):
        while self.running and rclpy.ok():
            try:
                position = self.sim.getObjectPosition(self.robotHandle, -1)
                orientation = self.sim.getObjectOrientation(self.robotHandle, -1)
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

    def _on_publish_odometry(self):
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

    def _on_publish_laserscan(self):
        timer_period_sec = 0.1
        laserscan_msg = LaserScan()
        laserscan_msg.header.frame_id = "laser_link"
        laserscan_msg.angle_increment = math.pi / (self.density * 180.0)
        laserscan_msg.angle_min = -90 * (math.pi/180)
        laserscan_msg.angle_max = (90* (math.pi/180)) - laserscan_msg.angle_increment
        laserscan_msg.time_increment = 0.0
        laserscan_msg.scan_time = timer_period_sec
        laserscan_msg.range_min = 0.055
        laserscan_msg.range_max = 10.0
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

    def _on_publish_docking(self):
        strength = Float32()
        angle = Float32()
        near = False

        if self._mode in (Mode.AUTODOCKING, Mode.CHARGING, Mode.CHARGED):
            near = random.random() > 0.7  # pretend we sometimes see the dock strong
            strength.data = 0.5 + (0.5 if near else 0.0)
            angle.data = 0.0 if near else random.uniform(-math.pi/4, math.pi/4)
        else:
            strength.data = random.uniform(0.0, 0.4)
            angle.data = random.uniform(-math.pi, math.pi)

        self._near_base = near
        self.pub_strength.publish(strength)
        self.pub_angle.publish(angle)

    def _on_publish_battery(self):
        if self._mode == Mode.CLEANING:
            self._battery = max(0.0, self._battery - 0.002)  # drains slowly
        elif self._mode == Mode.CHARGING:
            self._battery = min(1.0, self._battery + 0.004)  # charges faster

        bs = BatteryState()
        bs.percentage = float(self._battery)
        bs.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._mode == Mode.CHARGING
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.pub_batt.publish(bs)

    def _on_timer(self):
        self._on_publish_docking()
        self._on_publish_battery()
        self._on_publish_transform()
        self._on_publish_odometry()
       

        

    

def main():
    rclpy.init()
    node = MyRobot()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if node.should_exit():
                break
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down early')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
