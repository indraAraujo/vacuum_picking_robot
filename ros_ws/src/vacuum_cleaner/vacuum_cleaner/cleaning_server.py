#!/usr/bin/env python3
"""
cleaning_server
---------------
- Lifecycle: configurado e ativado pelo lifecycle_manager.
- Operação: só age quando o behaviour_server publica o modo CLEANING.
- No modo CLEANING, varre o ambiente em faixas paralelas (boustrophedon),
  com a estratégia do controlador do projeto 4 - roomba (ver coverage.py), e
  publica a velocidade em /myRobot/cmd_vel.

Os cinco sensores de proximidade do original vêm de uma de duas fontes
(parâmetro ``sensors``):

- ``map`` (padrão): raios no mapa (/map) a partir da pose exata da simulação
  (/myRobot/odom). Não precisa do lidar, que pode ficar desligado durante a
  limpeza (ver map_sensors.py);
- ``lidar``: os pontos do /scan.

Área de limpeza: ``/myRobot/cleaning/area`` (std_msgs/Float32MultiArray,
latched) traz retângulos ``[x_min, x_max, y_min, y_max, ...]`` no frame map, e
fora deles é parede virtual. É assim que "limpar a sala" fica só na sala: as
portas e as passagens abertas viram parede. Lista vazia libera a casa toda.
Se o robô estiver fora da área (o docking anda às cegas e pode empurrá-lo), a
parede virtual recua até ele; quando ele volta para dentro, ela volta ao lugar.

Segurança: as paredes da cena não têm colisão, então andar sem enxergar faria
o robô atravessá-las. Sem mapa (ou sem scan recente, na fonte ``lidar``), o
robô fica parado.
"""
import math
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from .modes import Mode
import tf2_ros
from .coverage import AXLE_X, Coverage, scan_points, virtual_sensors
from .map_sensors import Area, MapSensors, area_readings

#: Posição do laser em base_link (a mesma do static_transform_publisher).
LASER_X = 0.003
#: Scan mais velho que isto (tempo de simulação) faz o robô parar [s].
SCAN_TIMEOUT = 0.5
#: Folga da parede virtual quando o robô está fora da área [m].
AREA_SLACK = 0.10


class Cleaning(LifecycleNode):
    def __init__(self):
        super().__init__('cleaning_server')
        self.pub_cmd = None
        self._subs = []
        self._tick_timer = None
        self._mode = Mode.IDLE
        self._scan = None
        self._odom = None
        self._map = None
        self._area = None
        self._outside_warned = False
        self.declare_parameter('sensors', 'map')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_t = None
        self._blind_warned = False
        self.coverage = Coverage(log=lambda m: self.get_logger().info(m))
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        self.pub_cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', qos)
        # Estado da varredura (SEEK, LANE, CONTOUR, TURN, BACK), para depurar.
        self.pub_state = self.create_publisher(String, '/myRobot/cleaning/state', qos)
        self.source = self.get_parameter('sensors').value
        if self.source not in ('map', 'lidar'):
            self.get_logger().error(f'sensors:={self.source}? use map ou lidar')
            return TransitionCallbackReturn.FAILURE
        self._subs = [
            self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos),
            self.create_subscription(Odometry, '/myRobot/odom', self._on_odom, 10),
        ]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subs.append(self.create_subscription(
            Float32MultiArray, '/myRobot/cleaning/area', self._on_area, latched))
        if self.source == 'map':
            self._subs.append(self.create_subscription(
                OccupancyGrid, '/map', self._on_map, latched))
        else:
            self._subs.append(self.create_subscription(
                LaserScan, '/scan', self._on_scan, qos_profile_sensor_data))
        # O original age a cada passo da simulação (50 ms); aqui, no mesmo
        # ritmo, no relógio da simulação (use_sim_time).
        self._tick_timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'cleaning configured (varredura em faixas, sensores: {self.source})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        if self._tick_timer is not None and self._tick_timer.is_canceled():
            self._tick_timer.reset()
        self.get_logger().info('cleaning activated')
        return TransitionCallbackReturn.SUCCESS

    def _cancel_timer(self):
        if self._tick_timer is not None and not self._tick_timer.is_canceled():
            self._tick_timer.cancel()

    def _destroy_resources(self):
        if self._tick_timer is not None:
            self.destroy_timer(self._tick_timer)
            self._tick_timer = None
        if self.pub_cmd is not None:
            self.destroy_publisher(self.pub_cmd)
            self.pub_cmd = None
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []

    def on_deactivate(self, _):
        self.get_logger().info('cleaning deactivating: cancelling timers')
        self._cancel_timer()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('cleaning cleaning up resources')
        self._cancel_timer()
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('cleaning shutting down')
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

    # ---------------------------------------------------------------- entradas
    def _on_mode(self, msg: String):
        try:
            mode = Mode(msg.data)
        except Exception:
            mode = Mode.IDLE
        if mode == self._mode:
            return
        if mode == Mode.CLEANING:
            self.coverage.reset()
            self._last_t = None
            self.get_logger().info('limpeza: começando a varredura em faixas')
        elif self._mode == Mode.CLEANING:
            self._publish(0.0, 0.0)
            self.get_logger().info(f'limpeza interrompida (modo {mode.value})')
        self._mode = mode

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_map(self, msg: OccupancyGrid):
        self._map = MapSensors.from_msg(msg)

    def _on_area(self, msg: Float32MultiArray):
        try:
            self._area = Area.from_flat(msg.data)
        except ValueError as erro:
            self.get_logger().error(f'área de limpeza inválida: {erro}')
            return
        self._outside_warned = False
        if self._area is None:
            self.get_logger().info('área de limpeza: a casa toda')
        else:
            self.get_logger().info('área de limpeza: ' + ', '.join(
                f'x {x0:+.2f}..{x1:+.2f} y {y0:+.2f}..{y1:+.2f}'
                for x0, x1, y0, y1 in self._area.rects))

    def _area_at(self, x, y):
        """A área, alargada até o robô se ele estiver fora dela."""
        area = self._area
        if area is None:
            return None
        fora = area.follow(x, y, AREA_SLACK)
        if fora <= 0.0 and self._outside_warned:
            self.get_logger().info('o robô voltou para a área de limpeza')
            self._outside_warned = False
        elif fora > 0.0 and not self._outside_warned:
            self.get_logger().warn(
                f'o robô está {fora:.2f} m fora da área de limpeza: '
                'a parede virtual recua até ele')
            self._outside_warned = True
        return area

    # ------------------------------------------------------------------ ciclo
    def _publish(self, lin, rot):
        if self.pub_cmd is None:
            return
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(rot)
        self.pub_cmd.publish(twist)

    def _blind(self, reason):
        self._publish(0.0, 0.0)
        if not self._blind_warned:
            self.get_logger().warn(f'limpeza parada: {reason}')
            self._blind_warned = True

    def _map_pose(self, pose):
        """Pose de base_link no mapa: odometria composta com map->odom."""
        x, y = pose.position.x, pose.position.y
        theta = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        tx, ty = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        return tx + c * x - s * y, ty + s * x + c * y, theta + yaw

    def _readings(self, now):
        """Leituras dos sensores virtuais e a pose, ou None se estiver cego."""
        if self._odom is None:
            self._blind('esperando /myRobot/odom')
            return None
        pose = self._odom.pose.pose
        if self.source == 'map':
            if self._map is None:
                self._blind('esperando o mapa (/map)')
                return None
            x, y, theta = self._map_pose(pose) or (None, None, None)
            if x is None:
                self._blind('esperando a transformação map->odom')
                return None
            area = self._area_at(x, y)
            return self._map.readings(x, y, theta, area), x, y, theta
        if self._scan is None:
            self._blind('esperando /scan')
            return None
        scan_t = self._scan.header.stamp.sec + self._scan.header.stamp.nanosec * 1e-9
        if now - scan_t > SCAN_TIMEOUT:
            self._blind('o scan está atrasado')
            return None
        points = scan_points(self._scan.ranges, self._scan.angle_min,
                             self._scan.angle_increment, laser_x=LASER_X)
        if not points:
            self._blind('o scan está vazio (lidar desligado?)')
            return None
        theta = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        readings = virtual_sensors(points)
        no_mapa = self._map_pose(pose) if self._area is not None else None
        if no_mapa is not None:
            area = self._area_at(no_mapa[0], no_mapa[1])
            fronteira = area_readings(area, *no_mapa)
            readings = {k: min(v, fronteira[k]) for k, v in readings.items()}
        return readings, pose.position.x, pose.position.y, theta

    def _tick(self):
        if self._mode != Mode.CLEANING:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        lido = self._readings(now)
        if lido is None:
            return
        if self._blind_warned:
            self.get_logger().info('limpeza retomada: os sensores voltaram')
            self._blind_warned = False
        readings, bx, by, theta = lido

        # A varredura usa o centro do eixo das rodas, como no original.
        x = bx + AXLE_X * math.cos(theta)
        y = by + AXLE_X * math.sin(theta)
        dt = 0.05 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        lin, rot = self.coverage.step(now, dt, readings, x, y, theta)
        self._publish(lin, rot)
        self.pub_state.publish(String(data=self.coverage.state))


def main():
    rclpy.init()
    node = Cleaning()
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
