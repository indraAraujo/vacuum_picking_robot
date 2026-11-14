#!/usr/bin/env python3
"""vision: acha as bananas do chão na imagem do Kinect e publica onde estão.

Entra: ``/myRobot/kinect/rgb/image_raw``, ``.../depth/image_raw`` e
``.../camera_info`` (a ponte), mais a TF ``map -> kinect_optical``.

Sai: ``/bananas`` (geometry_msgs/PoseArray, frame map), o mesmo tópico e o
mesmo formato que o nó world publica a partir da simulação. Trocar um pelo
outro não muda nada no picker.

Cada quadro vira detecções pelo detect.py: pixels amarelos, profundidade e a
TF da câmera. O que a câmera vê num quadro é pouco (ela olha para a frente),
então o nó **guarda** as bananas que já viu, no frame map:

- uma detecção perto de uma banana conhecida atualiza aquela banana;
- uma detecção longe de todas vira uma banana nova;
- uma banana conhecida que devia aparecer no quadro (está no campo de visão,
  perto o bastante) e não apareceu ganha uma falta; depois de ``max_misses``
  faltas seguidas, ela é esquecida. É assim que somem as que o robô pegou.

Quadros em que o braço está na frente da câmera são descartados: nessa pose
ele tapa quase tudo.
"""
import math

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
import tf2_ros

from . import arm, detect

#: Nome do frame óptico da câmera (o mesmo da ponte).
CAMERA_FRAME = 'kinect_optical'
#: Comprimento da banana [m], para saber se ela caberia inteira na imagem.
BANANA_LENGTH = 0.21


def matrix_from_transform(t):
    """Matriz 3x4 de um geometry_msgs/TransformStamped."""
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    p = t.transform.translation
    return np.hstack([r, np.array([[p.x], [p.y], [p.z]])])


class Track:
    """Uma banana que o robô já viu, no frame map."""

    def __init__(self, x, y, z, yaw):
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.hits, self.misses = 1, 0

    def update(self, x, y, z, yaw, alpha=0.5):
        self.x += alpha * (x - self.x)
        self.y += alpha * (y - self.y)
        self.z += alpha * (z - self.z)
        # O rumo vale módulo 180 graus: a banana é simétrica.
        d = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
        if d > math.pi / 2:
            d -= math.pi
        elif d < -math.pi / 2:
            d += math.pi
        self.yaw = math.atan2(math.sin(self.yaw + alpha * d),
                              math.cos(self.yaw + alpha * d))
        self.hits += 1
        self.misses = 0


class Vision(Node):

    def __init__(self):
        super().__init__('vision')
        decl = self.declare_parameter
        decl('merge_dist', 0.30)       # detecção a até isto atualiza a banana [m]
        decl('max_misses', 3)          # faltas seguidas até esquecer
        decl('min_hits', 2)            # quadros até anunciar uma banana nova
        # Ao longe o borrão tem poucos pixels e a estimativa piora; o robô
        # chega mais perto e vê de novo.
        decl('max_range', 2.5)         # profundidade máxima aproveitada [m]
        decl('min_area', 60)           # pixels mínimos do borrão
        decl('floor_max', 0.10)        # altura máxima para estar no chão [m]
        decl('basket_radius', 0.35)    # as de dentro do cesto não contam [m]
        decl('check_range', 2.5)       # só cobra falta até esta distância [m]
        self.tracks = []
        self.info = None
        self.depth = None          # (instante, imagem)
        self.rgb = None            # (instante, mensagem)
        self.last = None           # instante do último par processado
        self.basket = None
        self.blocked = False           # braço na frente da câmera
        self.frames = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, '/myRobot/kinect/camera_info',
                                 self.on_info, latched)
        self.create_subscription(Image, '/myRobot/kinect/depth/image_raw',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, '/myRobot/kinect/rgb/image_raw',
                                 self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/basket', self.on_basket, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.pub = self.create_publisher(PoseArray, '/bananas', 10)
        self.debug_pub = self.create_publisher(Image, '/bananas/debug',
                                               qos_profile_sensor_data)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.get_logger().info('visão pronta: esperando as imagens do Kinect')

    # ------------------------------------------------------------- entradas
    def on_info(self, msg):
        self.info = msg

    def on_basket(self, msg):
        self.basket = (msg.pose.position.x, msg.pose.position.y)

    def on_joints(self, msg):
        if len(msg.position) >= 5:
            y, z = arm.fingertip(list(msg.position[:5]))
            # Na pose de pegar, o braço desce na frente e tapa a câmera.
            self.blocked = y > 0.0 and z < 0.45

    def on_depth(self, msg):
        if msg.encoding != '32FC1':
            return
        self.depth = (Time.from_msg(msg.header.stamp).nanoseconds,
                      np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width))
        self.process()

    def on_rgb(self, msg):
        if msg.encoding != 'rgb8':
            return
        self.rgb = (Time.from_msg(msg.header.stamp).nanoseconds, msg)
        self.process()

    def process(self):
        """Roda quando a cor e a profundidade do mesmo instante já chegaram."""
        if self.info is None or self.depth is None or self.rgb is None:
            return
        if abs(self.depth[0] - self.rgb[0]) > 1_000_000:     # ainda não é o par
            return
        if self.rgb[0] == self.last:
            return
        self.last = self.rgb[0]
        msg = self.rgb[1]
        t = Time.from_msg(msg.header.stamp)
        if self.blocked:
            return
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, t)
        except tf2_ros.TransformException:
            try:
                tf = self.tf.lookup_transform('map', msg.header.frame_id, Time())
            except tf2_ros.TransformException as erro:
                self.get_logger().warn(f'sem a TF da câmera: {erro}', once=True)
                return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        p = self.get_parameter
        to_map = matrix_from_transform(tf)
        achadas = detect.detect(
            rgb, self.depth[1], self.info.k[0], self.info.k[4], self.info.k[2],
            self.info.k[5], to_map, min_area=int(p('min_area').value),
            max_range=float(p('max_range').value),
            floor_max=float(p('floor_max').value),
            exclude=None if self.basket is None
            else (self.basket[0], self.basket[1], float(p('basket_radius').value)))
        achadas = [d for d in achadas if not self.in_basket(d)]
        self.update(achadas, to_map)
        self.publish()
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f'primeiro quadro processado: {len(achadas)} bananas na imagem')
        if self.debug_pub.get_subscription_count():
            self.publish_debug(msg, rgb, achadas, to_map)

    # -------------------------------------------------------------- memória
    def in_basket(self, d):
        if self.basket is None:
            return False
        raio = float(self.get_parameter('basket_radius').value)
        return math.hypot(d[0] - self.basket[0], d[1] - self.basket[1]) < raio

    def visible(self, track, to_map):
        """A banana conhecida devia aparecer inteira neste quadro?

        A margem é meia banana na imagem: um borrão que encosta na borda está
        cortado, e o detect.py o descarta. Sem essa folga, a banana que sai
        pela borda contaria falta e seria esquecida à toa.
        """
        r, t = to_map[:, :3], to_map[:, 3]
        p = r.T @ (np.array([track.x, track.y, track.z]) - t)   # para o óptico
        if p[2] <= 0.2 or p[2] > float(self.get_parameter('check_range').value):
            return False
        fx, fy = self.info.k[0], self.info.k[4]
        u = fx * p[0] / p[2] + self.info.k[2]
        v = fy * p[1] / p[2] + self.info.k[5]
        margem = 0.5 * BANANA_LENGTH * fx / p[2] + 10
        return (margem <= u < self.info.width - margem
                and margem <= v < self.info.height - margem)

    def update(self, achadas, to_map):
        merge = float(self.get_parameter('merge_dist').value)
        vistas = set()
        for (x, y, z, yaw) in achadas:
            perto, dm = None, merge
            for k, tr in enumerate(self.tracks):
                d = math.hypot(tr.x - x, tr.y - y)
                if d < dm and k not in vistas:
                    perto, dm = k, d
            if perto is None:
                self.tracks.append(Track(x, y, z, yaw))
                vistas.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[perto]
                tr.update(x, y, z, yaw)
                vistas.add(perto)
                if tr.hits == int(self.get_parameter('min_hits').value):
                    self.get_logger().info(
                        f'banana em ({tr.x:+.2f}, {tr.y:+.2f}), rumo '
                        f'{math.degrees(tr.yaw):+.0f} graus')
        limite = int(self.get_parameter('max_misses').value)
        restam = []
        for k, tr in enumerate(self.tracks):
            if k not in vistas and self.visible(tr, to_map):
                tr.misses += 1
                if tr.misses >= limite:
                    if tr.hits >= int(self.get_parameter('min_hits').value):
                        self.get_logger().info(
                            f'a banana de ({tr.x:+.2f}, {tr.y:+.2f}) não está mais lá')
                    continue
            restam.append(tr)
        self.tracks = restam

    def publish(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        minimo = int(self.get_parameter('min_hits').value)
        for tr in self.tracks:
            if tr.hits < minimo:
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = tr.x, tr.y, tr.z
            pose.orientation.z = math.sin(tr.yaw / 2.0)
            pose.orientation.w = math.cos(tr.yaw / 2.0)
            msg.poses.append(pose)
        self.pub.publish(msg)

    def publish_debug(self, src, rgb, achadas, to_map):
        import cv2
        img = rgb.copy()
        for (x, y, z, yaw) in achadas:
            p = to_map[:, :3].T @ (np.array([x, y, z]) - to_map[:, 3])
            u = int(self.info.k[0] * p[0] / p[2] + self.info.k[2])
            v = int(self.info.k[4] * p[1] / p[2] + self.info.k[5])
            cv2.circle(img, (u, v), 8, (255, 0, 0), 2)
            cv2.putText(img, f'{math.degrees(yaw):.0f}', (u + 10, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(img, f'{len(achadas)} vistas, {len(self.tracks)} conhecidas',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out = Image()
        out.header = src.header
        out.height, out.width = img.shape[0], img.shape[1]
        out.encoding = 'rgb8'
        out.step = img.shape[1] * 3
        out.data = np.ascontiguousarray(img).tobytes()
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
