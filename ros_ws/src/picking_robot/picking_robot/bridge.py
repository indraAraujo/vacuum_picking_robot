#!/usr/bin/env python3
"""picking_bridge: ponte ROS 2 do robô com garra (simulation/scene_house_arm.ttt).

Entradas:

- ``/myRobot/cmd_vel`` (geometry_msgs/Twist): velocidade da base. Sem comando
  por ``cmd_timeout`` segundos de simulação, as rodas param;
- ``/myRobot/arm/command`` (std_msgs/Float64MultiArray): alvo das 5 juntas do
  braço [rad], presos aos limites (ver arm.py);
- ``/myRobot/arm/pose`` (std_msgs/String): uma pose pronta: repouso, alto,
  chao ou carregar;
- ``/myRobot/gripper/command`` (std_msgs/Bool): true fecha a garra, false abre.

Saídas:

- ``/clock``: o tempo da simulação (use ``use_sim_time:=true`` nos outros nós);
- ``/myRobot/odom`` e a TF ``odom -> base_link``: a pose exata da simulação,
  com a origem onde o robô começou e a frente em +x;
- ``/joint_states``: as 5 juntas do braço [rad] e os 2 dedos [m];
- ``/myRobot/gripper/holding`` (std_msgs/Bool): se a garra segura algo;
- ``/scan``: um scan vazio (sem nenhum eco) a cada 0,1 s de simulação. A cena
  não tem laser, e o Nav2 (parâmetros do vacuum_cleaner) espera um: vazio, ele
  não marca obstáculos, mas mantém o collision_monitor ativo;
- o Kinect: ``/myRobot/kinect/rgb/image_raw`` (rgb8),
  ``/myRobot/kinect/depth/image_raw`` (32FC1, em metros),
  ``/myRobot/kinect/camera_info`` e a TF ``base_link -> kinect_optical``.

Renderizar o Kinect custa ~35 ms por quadro e segura a simulação, então a
ponte só renderiza enquanto alguém estiver assinando as imagens (o nó vision,
o rqt_image_view). Sem ninguém ouvindo, nada é renderizado.

Cada leitura e cada comando são uma chamada só ao script
``/myRobot/rosInterface`` da cena (ver build_scene.py).
"""
import math
import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import TransformStamped, Twist
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Bool, Float64MultiArray, String
import tf2_ros

from . import arm

WHEEL_BASE = 0.20
WHEEL_RADIUS = 0.05
#: Na cena, a frente do robô é o +y do /myRobot; em base_link, +x.
FORWARD_YAW_OFFSET = math.pi / 2.0
#: As juntas das rodas giram ao contrário do sentido de avanço.
MOTOR_SIGN = -1.0
#: Período do scan vazio [s de simulação].
SCAN_PERIOD = 0.1
#: Frame óptico do Kinect (x para a direita, y para baixo, z para a frente).
CAMERA_FRAME = 'kinect_optical'


def quaternion(r):
    """Quaternio (x, y, z, w) de uma matriz de rotação 3x3."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i], q[j], q[k] = 0.25 * s, (r[j][i] + r[i][j]) / s, (r[k][i] + r[i][k]) / s
    return (q[0], q[1], q[2], (r[k][j] - r[j][k]) / s)


class PickingBridge(Node):

    def __init__(self):
        super().__init__('picking_bridge')
        self.declare_parameter('sim_port', 23000)
        self.declare_parameter('cmd_timeout', 0.6)
        # Quadros por segundo de simulação do Kinect, quando alguém assina as
        # imagens. Cada quadro custa ~35 ms e segura a simulação.
        self.declare_parameter('kinect_rate', 5.0)
        port = int(self.get_parameter('sim_port').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # O cliente ZMQ não pode ser compartilhado entre threads: um para os
        # comandos (executor) e outro para a leitura (thread).
        self.sim = RemoteAPIClient(port=port).getObject('sim')
        self.robot = self.sim.getObject('/myRobot')
        self.iface = self.sim.getObject('/myRobot/rosInterface', {'noError': True})
        if self.iface < 0:
            raise SystemExit(
                'a cena aberta no CoppeliaSim não tem /myRobot/rosInterface: '
                'abra simulation/scene_house_arm.ttt (ou gere-a com '
                '"ros2 run picking_robot build_scene")')
        self.kinect = {}
        for nome in ('rgb', 'depth'):
            h = self.sim.getObject(f'/myRobot/kinect/{nome}', {'noError': True})
            if h >= 0:
                self.sim.setExplicitHandling(h, 1)   # só renderiza quando pedimos
                self.kinect[nome] = h
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()

        self.lock = threading.Lock()
        self.sim_time = 0.0
        self.last_cmd_time = None
        self.wheels = (None, None)
        self.origin = None          # (x0, y0, yaw0) da cena, na primeira leitura
        self.prev = None            # (t, x, y, theta), para a velocidade

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/myRobot/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.holding_pub = self.create_publisher(Bool, '/myRobot/gripper/holding', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.scan = LaserScan()
        self.scan.header.frame_id = 'laser_link'
        self.scan.angle_min, self.scan.angle_max = -math.pi / 2, math.pi / 2
        self.scan.angle_increment = math.pi / 180.0
        self.scan.range_min, self.scan.range_max = 0.055, 10.0
        self.scan.ranges = [float('inf')] * 181
        self.last_scan = -1.0
        self.tf = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, '/myRobot/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Float64MultiArray, '/myRobot/arm/command', self.on_arm, 10)
        self.create_subscription(String, '/myRobot/arm/pose', self.on_pose, 10)
        self.create_subscription(Bool, '/myRobot/gripper/command', self.on_gripper, 10)
        self.create_timer(0.1, self.check_timeout)

        self.running = True
        self.reader_sim = RemoteAPIClient(port=port).getObject('sim')
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        if len(self.kinect) == 2:
            self.setup_camera(port)
        self.get_logger().info(
            'ponte pronta: /myRobot/cmd_vel, /myRobot/arm/command, /myRobot/arm/pose, '
            '/myRobot/gripper/command')

    # ------------------------------------------------------------- comandos
    def call(self, func, *args):
        return self.sim.callScriptFunction(func, self.iface, *args)

    def set_wheels(self, linear, angular):
        left = MOTOR_SIGN * (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right = MOTOR_SIGN * (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        if (left, right) != self.wheels:
            self.call('setWheels', left, right)
            self.wheels = (left, right)

    def on_cmd_vel(self, msg):
        with self.lock:
            self.last_cmd_time = self.sim_time
        self.set_wheels(msg.linear.x, msg.angular.z)

    def check_timeout(self):
        with self.lock:
            parado = (self.last_cmd_time is None
                      or self.sim_time - self.last_cmd_time > self.cmd_timeout)
        if parado:
            self.set_wheels(0.0, 0.0)

    def on_arm(self, msg):
        try:
            alvo = arm.clamp(msg.data)
        except ValueError as erro:
            self.get_logger().warn(str(erro))
            return
        self.call('setArm', alvo)

    def on_pose(self, msg):
        alvo = arm.POSES.get(msg.data.strip())
        if alvo is None:
            self.get_logger().warn(
                f'pose "{msg.data}" desconhecida; tenho: ' + ', '.join(arm.POSES))
            return
        self.get_logger().info(f'braço: pose {msg.data}')
        self.call('setArm', alvo)

    def on_gripper(self, msg):
        self.get_logger().info('garra: ' + ('fechando' if msg.data else 'abrindo'))
        self.call('setGripper', bool(msg.data))

    # --------------------------------------------------------------- leitura
    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                pos, ori, t, q, held = self.reader_sim.callScriptFunction('snapshot', self.iface)
            except Exception as erro:
                self.get_logger().error(f'não consegui ler a cena: {erro}')
                time.sleep(0.5)
                continue
            if self.prev is not None and t <= self.prev[0]:
                time.sleep(0.002)
                continue
            try:
                self.publish(pos, ori, t, q, held)
            except Exception:
                if not rclpy.ok():      # Ctrl+C: o contexto fechou no meio
                    break
                raise

    def publish(self, pos, ori, t, q, held):
        heading = ori[2] + FORWARD_YAW_OFFSET
        if self.origin is None:
            self.origin = (pos[0], pos[1], heading)
        x0, y0, yaw0 = self.origin
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = pos[0] - x0, pos[1] - y0
        x, y = c * dx + s * dy, -s * dx + c * dy
        theta = math.atan2(math.sin(heading - yaw0), math.cos(heading - yaw0))
        v = w = 0.0
        if self.prev is not None:
            pt, px, py, pth = self.prev
            dt = t - pt
            v = ((x - px) * math.cos(pth) + (y - py) * math.sin(pth)) / dt
            w = math.atan2(math.sin(theta - pth), math.cos(theta - pth)) / dt
        self.prev = (t, x, y, theta)
        with self.lock:
            self.sim_time = t

        stamp = Time(nanoseconds=int(t * 1e9)).to_msg()
        self.clock_pub.publish(Clock(clock=stamp))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        js.name = arm.ARM_JOINTS + arm.GRIPPER_JOINTS
        js.position = [float(v) for v in q]
        self.joint_pub.publish(js)
        self.holding_pub.publish(Bool(data=held is not None and held >= 0))
        if t - self.last_scan >= SCAN_PERIOD:
            self.last_scan = t
            self.scan.header.stamp = stamp
            self.scan_pub.publish(self.scan)


    # ---------------------------------------------------------------- câmera
    def setup_camera(self, port):
        """Mede a pose do Kinect, publica a TF dele e sobe a thread da câmera."""
        rgb = self.kinect['rgb']
        w = self.sim.getObjectInt32Param(rgb, self.sim.visionintparam_resolution_x)
        h = self.sim.getObjectInt32Param(rgb, self.sim.visionintparam_resolution_y)
        fov = self.sim.getObjectFloatParam(rgb, self.sim.visionfloatparam_perspective_angle)
        # O ângulo do CoppeliaSim vale para o lado maior da imagem.
        f = (max(w, h) / 2.0) / math.tan(fov / 2.0)
        self.cam_size = (w, h)
        self.rgb_pub = self.create_publisher(
            Image, '/myRobot/kinect/rgb/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, '/myRobot/kinect/depth/image_raw', qos_profile_sensor_data)
        info_pub = self.create_publisher(
            CameraInfo, '/myRobot/kinect/camera_info',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        info = CameraInfo()
        info.header.frame_id = CAMERA_FRAME
        info.width, info.height = w, h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info_pub.publish(info)
        self.cam_info = info

        # TF base_link -> kinect_optical. Em base_link a frente é +x, e no
        # /myRobot da cena é +y; o sensor do CoppeliaSim tem x para a esquerda
        # e y para cima, e o frame óptico do ROS, x para a direita e y para
        # baixo (meia volta em torno do eixo z do sensor).
        m = np.array(self.sim.getObjectMatrix(rgb, self.robot)).reshape(3, 4)
        robot_para_base = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        rot = robot_para_base @ m[:, :3] @ np.diag([-1.0, -1.0, 1.0])
        pos = robot_para_base @ m[:, 3]
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = CAMERA_FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos
        q = quaternion(rot.tolist())
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = q
        self.static_tf = tf2_ros.StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        self.camera = threading.Thread(target=self.camera_loop, args=(port,), daemon=True)
        self.camera.start()
        self.get_logger().info(
            f'Kinect {w}x{h} (f={f:.0f} px) em {CAMERA_FRAME}; renderizo enquanto '
            'alguém assinar /myRobot/kinect/rgb/image_raw ou .../depth/image_raw')

    def camera_loop(self, port):
        sim = RemoteAPIClient(port=port).getObject('sim')   # cliente próprio
        rgb, depth = self.kinect['rgb'], self.kinect['depth']
        w, h = self.cam_size
        periodo = 1.0 / max(0.1, float(self.get_parameter('kinect_rate').value))
        ultimo = -1e9
        while self.running and rclpy.ok():
            if not (self.rgb_pub.get_subscription_count()
                    or self.depth_pub.get_subscription_count()):
                time.sleep(0.2)
                continue
            with self.lock:
                t = self.sim_time
            if t - ultimo < periodo:
                time.sleep(0.01)
                continue
            ultimo = t
            try:
                sim.handleVisionSensor(rgb)
                sim.handleVisionSensor(depth)
                cor, _ = sim.getVisionSensorImg(rgb)
                prof, _ = sim.getVisionSensorDepth(depth, 1)
            except Exception as erro:
                if not rclpy.ok():
                    break
                self.get_logger().warn(f'não consegui ler o Kinect: {erro}')
                time.sleep(0.5)
                continue
            # O CoppeliaSim entrega a imagem de baixo para cima; o ROS, de cima
            # para baixo.
            cor = np.frombuffer(cor, np.uint8).reshape(h, w, 3)[::-1]
            prof = np.frombuffer(prof, np.float32).reshape(h, w)[::-1]
            stamp = Time(nanoseconds=int(t * 1e9)).to_msg()
            try:
                self.rgb_pub.publish(self.image(stamp, cor, 'rgb8', w * 3))
                self.depth_pub.publish(self.image(stamp, prof, '32FC1', w * 4))
                self.cam_info.header.stamp = stamp
            except Exception:
                if not rclpy.ok():
                    break
                raise

    def image(self, stamp, data, encoding, step):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME
        msg.height, msg.width = data.shape[0], data.shape[1]
        msg.encoding = encoding
        msg.step = step
        msg.data = np.ascontiguousarray(data).tobytes()
        return msg


def main():
    rclpy.init()
    node = PickingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        try:
            RemoteAPIClient(port=int(node.get_parameter('sim_port').value)) \
                .getObject('sim').callScriptFunction('setWheels', node.iface, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
