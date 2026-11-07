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
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool
from rclpy.time import Time
from .modes import Mode
from coppeliasim_zmqremoteapi_client import *

WHEEL_BASE = 0.20
WHEEL_RADIUS = 0.05
# Na cena, a frente do robô é o eixo +Y do objeto /myRobot; em base_link é +X.
FORWARD_YAW_OFFSET = math.pi / 2.0
# As juntas giram ao contrário do sentido de avanço do robô.
MOTOR_SIGN = -1.0
# Tempo de simulação sem comando até parar as rodas.
CMD_TIMEOUT = 0.6
# Período do scan vazio publicado com o laser desligado [s de simulação].
EMPTY_SCAN_PERIOD = 0.1

class MyRobot(LifecycleNode):
    def __init__(self):
        super().__init__('myrobot')

        # Porta da ZeroMQ Remote API. Outra porta permite rodar uma segunda
        # instância do CoppeliaSim (por exemplo, sem janela, para testes).
        self.declare_parameter('sim_port', 23000)
        self.sim_port = int(self.get_parameter('sim_port').value)
        self.client = RemoteAPIClient(port=self.sim_port)
        self.sim = self.client.getObject('sim')

        self._battery = 1.0
        self._battery_t = None
        # Descarga limpando e carga na base, em fração por segundo de simulação.
        self.declare_parameter('battery_drain', 0.01)
        self.declare_parameter('battery_charge', 0.02)
        self.battery_drain = float(self.get_parameter('battery_drain').value)
        self.battery_charge = float(self.get_parameter('battery_charge').value)
        self._near_base = False
        self._last_cmd = Twist()
        self._mode = Mode.IDLE
        self._timer = None
        self._motor_timer = None
        self.pub_strength = None
        self.pub_angle = None
        self.pub_batt = None
        self.sub_cmd = None
        self.sub_mode = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

        self.robotHandle = self.sim.getObject('/myRobot')

        self.left_motor = self.sim.getObject('/myRobot/leftMotor')
        self.right_motor = self.sim.getObject('/myRobot/rightMotor')
        self._wheel_cmd = (None, None)
        self.last_cmd_time = None

        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        position = self.sim.getObjectPosition(self.robotHandle, -1)
        orientation = self.sim.getObjectOrientation(self.robotHandle, -1)
        self.x0, self.y0 = position[0], position[1]
        self.yaw0 = orientation[2] + FORWARD_YAW_OFFSET

        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_sim_time = self.sim.getSimulationTime()
        self.running = True

        self.sim_time = self.last_sim_time

        self.odom_publisher = self.create_publisher(Odometry, '/myRobot/odom', 10)
        self.laserscan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        # A simulação pode rodar bem abaixo do tempo real, então todo o ROS
        # (SLAM e Nav2) segue o relógio dela.
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)

        self.declare_parameter('scanning_density', 1.0)
        self.density = min(5.0, max(
            0.1, float(self.get_parameter('scanning_density').value)))
        laser_handle = self.sim.getObject('/myRobot/LaserScanner_2D')
        self.laser_script = self.sim.getScript(
            self.sim.scripttype_childscript, laser_handle)
        self.sim.setScriptSimulationParameter(
            self.laser_script, 'scanningDensity', self.density)

        # O laser é o que mais pesa na simulação. Este serviço liga e desliga
        # o script dele na cena (o modo de voz desliga). Ao subir, ele é
        # sempre religado, caso uma sessão anterior o tenha deixado desligado.
        self.lidar_enabled = True
        self._last_empty_scan = -1.0
        self._set_laser_script(True)
        self.create_service(SetBool, '/myRobot/lidar/enable', self._on_lidar_enable)

        self.laserscan_msg = LaserScan()
        self.laserscan_msg.header.frame_id = 'laser_link'
        self.laserscan_msg.angle_min = -math.pi / 2.0
        self.laserscan_msg.angle_increment = math.pi / (self.density * 180.0)
        self.laserscan_msg.range_min = 0.055
        self.laserscan_msg.range_max = 10.0

        # O cliente ZMQ não pode ser compartilhado entre threads.
        self.reader_client = RemoteAPIClient(port=self.sim_port)
        self.reader_sim = self.reader_client.getObject('sim')
        self.reader = threading.Thread(target=self._on_robot_pose, daemon=True)
        self.reader.start()

    # ---------------- Lidar ----------------
    def _set_laser_script(self, enabled):
        self.sim.setScriptInt32Param(
            self.laser_script, self.sim.scriptintparam_enabled, 1 if enabled else 0)

    def _on_lidar_enable(self, req, resp):
        try:
            self._set_laser_script(req.data)
        except Exception as e:
            resp.success = False
            resp.message = f'não consegui mudar o laser na cena: {e}'
            return resp
        # Desligado, o script deixa o último sinal de distâncias na cena; sem
        # esta flag, o /scan continuaria saindo com leituras velhas. Com ela,
        # sai um scan vazio no lugar (ver _on_publish_empty_scan).
        self.lidar_enabled = req.data
        estado = 'ligado' if req.data else 'desligado'
        self.get_logger().info(f'lidar {estado}')
        resp.success = True
        resp.message = f'lidar {estado}'
        return resp

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
        self._motor_timer = self.create_timer(0.05, self._on_send_velocity)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        for timer in (self._timer, self._motor_timer):
            if timer is not None and timer.is_canceled():
                timer.reset()
        self.get_logger().info('myrobot activated')
        return TransitionCallbackReturn.SUCCESS

    def _cancel_timer(self):
        for timer in (self._timer, self._motor_timer):
            if timer is not None and not timer.is_canceled():
                timer.cancel()
        self._set_wheels(0.0, 0.0)

    def _destroy_resources(self):
        for attr in ('_timer', '_motor_timer'):
            timer = getattr(self, attr)
            if timer is not None:
                self.destroy_timer(timer)
                setattr(self, attr, None)
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
        with self.lock:
            self.last_cmd_time = self.sim_time
        # Aplica já, sem esperar o timer: o timer é de relógio (50 ms), e com a
        # simulação mais rápida que o tempo real isso virava 0,25 s ou mais de
        # simulação com a velocidade antiga, o bastante para o robô passar do
        # ponto de parada e raspar na parede. O timer fica para o timeout.
        self._on_send_velocity()

    def _set_wheels(self, left, right):
        if (left, right) == self._wheel_cmd:
            return
        self.sim.setJointTargetVelocity(self.left_motor, left)
        self.sim.setJointTargetVelocity(self.right_motor, right)
        self._wheel_cmd = (left, right)

    def _on_send_velocity(self):
        with self.lock:
            stale = (self.last_cmd_time is None
                     or self.sim_time - self.last_cmd_time > CMD_TIMEOUT)
        linear = 0.0 if stale else self._last_cmd.linear.x
        angular = 0.0 if stale else self._last_cmd.angular.z

        left = MOTOR_SIGN * (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right = MOTOR_SIGN * (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        self._set_wheels(left, right)

    def _on_robot_pose(self):
        sim = self.reader_sim
        while self.running and rclpy.ok():
            try:
                position = sim.getObjectPosition(self.robotHandle, -1)
                orientation = sim.getObjectOrientation(self.robotHandle, -1)
                sim_time = sim.getSimulationTime()
            except Exception as e:
                self.get_logger().error(f'Failed to read CoppeliaSim state: {e}')
                time.sleep(0.5)
                continue

            dt = sim_time - self.last_sim_time
            if dt <= 0.0:
                time.sleep(0.01)
                continue

            with self.lock:
                dx = position[0] - self.x0
                dy = position[1] - self.y0
                c = math.cos(self.yaw0)
                s = math.sin(self.yaw0)
                x = c * dx + s * dy
                y = -s * dx + c * dy
                heading = orientation[2] + FORWARD_YAW_OFFSET
                theta = math.atan2(
                    math.sin(heading - self.yaw0),
                    math.cos(heading - self.yaw0))

                dtheta = math.atan2(
                    math.sin(theta - self.theta),
                    math.cos(theta - self.theta))
                self.v = (
                    (x - self.x) * math.cos(self.theta)
                    + (y - self.y) * math.sin(self.theta)) / dt
                self.w = dtheta / dt

                self.x = x
                self.y = y
                self.theta = theta
                self.last_sim_time = sim_time
                self.sim_time = sim_time

            stamp = Time(nanoseconds=int(sim_time * 1e9)).to_msg()
            clock = Clock()
            clock.clock = stamp
            self.clock_publisher.publish(clock)
            self._on_publish_odometry(stamp)
            if self.lidar_enabled:
                self._on_publish_laserscan(sim, stamp)
            elif sim_time - self._last_empty_scan >= EMPTY_SCAN_PERIOD:
                self._last_empty_scan = sim_time
                self._on_publish_empty_scan(stamp)

    def _on_publish_odometry(self, stamp):
        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
            v = self.v
            w = self.w

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _on_publish_laserscan(self, sim, stamp):
        distance = sim.getFloatArrayProperty(
            self.robotHandle, 'signal.distSignal', {'noError': True})
        if not distance:
            return

        msg = self.laserscan_msg
        msg.header.stamp = stamp
        msg.ranges = distance
        msg.angle_max = (
            msg.angle_min + (len(distance) - 1) * msg.angle_increment)
        self.laserscan_publisher.publish(msg)

    def _on_publish_empty_scan(self, stamp):
        """Scan sem nenhum eco, enquanto o laser está desligado.

        Não marca nem limpa nada nos costmaps (todas as leituras são infinitas)
        e o SLAM está pausado, mas o horário dele mantém viva a TF map -> odom,
        que o SLAM carimba com o horário do último scan.
        """
        msg = self.laserscan_msg
        n = int(round(180.0 * self.density)) + 1
        msg.header.stamp = stamp
        msg.ranges = [float('inf')] * n
        msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        self.laserscan_publisher.publish(msg)

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
        # Pelo tempo de simulação, não pelo relógio: a cena roda abaixo do
        # tempo real, e pelo relógio a bateria acabava em ~30 s de simulação.
        # Mesmas taxas por segundo de antes (1 %/s limpando, 2 %/s carregando).
        with self.lock:
            now = self.sim_time
        dt = 0.0 if self._battery_t is None else max(0.0, now - self._battery_t)
        self._battery_t = now
        if self._mode == Mode.CLEANING:
            self._battery = max(0.0, self._battery - self.battery_drain * dt)
        elif self._mode == Mode.CHARGING:
            self._battery = min(1.0, self._battery + self.battery_charge * dt)

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
