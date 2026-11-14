#!/usr/bin/env python3
"""world: as bananas pegáveis, o cesto e o mapa, a partir da simulação.

- Ao subir, espalha ``count`` bananas pelo chão livre da casa (``/pickBanana``
  na cena). Cada uma é uma caixa invisível com física (cai, quica, assenta no
  cesto) com o modelo da banana por cima, só visual. Elas não colidem com o
  robô (ver build_scene.py). Ao sair, tira todas da cena.
- Publica onde estão as bananas que ainda estão no chão: ``/bananas``
  (geometry_msgs/PoseArray, frame map). A posição é o meio da banana (ela é
  curva: não é o centro da caixa), e o ângulo é o do comprimento dela.

  Este é o "sensor perfeito": lê a posição exata da simulação. Um nó de
  visão (Kinect) pode publicar no mesmo tópico, no mesmo formato, e o resto
  do sistema não muda. Para isso, rode com ``publish_truth:=false``.
- Publica o cesto: ``/basket`` (geometry_msgs/PoseStamped, frame map,
  latched) e o raio dele no parâmetro do picker.
- Publica ``/map``: o mapa da casa (maps/casa.yaml do vacuum_cleaner) com o
  cesto marcado como obstáculo, para o Nav2 desviar dele.
- Conta as bananas no cesto: ``/bananas/delivered`` (std_msgs/Int32).
"""
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32
from vacuum_cleaner.debris import FloorMap, read_map_yaml, read_pgm, sample_spots

ALIAS = 'pickBanana'
#: Caixa de colisão da banana [m] e massa [kg] (a do modelo).
BOX = [0.20, 0.10, 0.034]
MASS = 0.2
#: Meio da banana no referencial dela: o modelo é curvo e o meio fica fora do
#: centro da caixa (medido na malha).
GRASP_OFFSET = [0.0, -0.031, 0.0]
#: Bit da máscara global de colisão das bananas (o robô não o tem).
BANANA_BIT = 0x0200


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class World(Node):

    def __init__(self):
        super().__init__('world')
        vc = get_package_share_directory('vacuum_cleaner')
        self.declare_parameter('sim_port', 23000)
        self.declare_parameter('count', 8)
        self.declare_parameter('seed', -1)
        self.declare_parameter('map', os.path.join(vc, 'maps', 'casa.yaml'))
        self.declare_parameter('model', os.path.join(vc, 'models', 'banana.ttm'))
        # Folga das paredes: o robô precisa chegar à banana de frente.
        self.declare_parameter('clearance', 0.45)
        self.declare_parameter('scene_origin', [3.55, 0.775])
        self.declare_parameter('publish_truth', True)
        # Posições fixas [x1, y1, x2, y2, ...] no map, no lugar do sorteio
        # (para repetir uma situação ou montar uma demonstração).
        self.declare_parameter('positions', [0.0])
        p = lambda n: self.get_parameter(n).value
        self.ox, self.oy = p('scene_origin')
        seed = int(p('seed'))
        self.rng = random.Random(None if seed < 0 else seed)
        self.port = int(p('sim_port'))

        self.sim = RemoteAPIClient(port=self.port).getObject('sim')
        self.iface = self.sim.getObject('/myRobot/rosInterface', {'noError': True})
        basket = self.sim.getObject('/bananaBasket', {'noError': True})
        if self.iface < 0 or basket < 0:
            raise SystemExit('a cena aberta não é simulation/scene_house_arm.ttt (atual): '
                             'falta /myRobot/rosInterface ou /bananaBasket')
        bx, by, _ = self.sim.getObjectPosition(basket, -1)
        size = self.sim.getShapeBB(basket)[0]
        self.basket = (bx - self.ox, by - self.oy)
        self.basket_r = size[0] / 2.0
        with open(p('model'), 'rb') as f:
            self.model = f.read()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.basket_pub = self.create_publisher(PoseStamped, '/basket', latched)
        self.delivered_pub = self.create_publisher(Int32, '/bananas/delivered', latched)
        self.truth = bool(p('publish_truth'))
        self.bananas_pub = self.create_publisher(PoseArray, '/bananas', 10)

        self.publish_map(p('map'))
        self.publish_basket()
        self.handles = []
        self.delivered = -1
        self.remove_leftovers()
        self.spawn(FloorMap.from_yaml(p('map')), int(p('count')), float(p('clearance')))
        self.create_timer(0.5, self.tick)

    # ------------------------------------------------------------------ mapa
    def publish_map(self, yaml_path):
        """O mapa da casa com o cesto marcado como ocupado."""
        info = read_map_yaml(yaml_path)
        w, h, px = read_pgm(info['image'])
        res = info['resolution']
        ox, oy = info['origin'][0], info['origin'][1]
        data = []
        for j in range(h):                      # linha 0 do grid = embaixo
            for v in px[(h - 1 - j) * w:(h - j) * w]:
                occ = v / 255.0 if info['negate'] else (255 - v) / 255.0
                data.append(100 if occ > 0.65 else 0 if occ < info['free_thresh'] else -1)
        cx, cy = self.basket
        r = self.basket_r + 0.02
        for j in range(h):
            for i in range(w):
                x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
                if math.hypot(x - cx, y - cy) <= r:
                    data[j * w + i] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width, msg.info.height = w, h
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        self.map_pub.publish(msg)

    def publish_basket(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = self.basket
        msg.pose.orientation.w = 1.0
        self.basket_pub.publish(msg)
        self.get_logger().info(
            f'cesto em ({self.basket[0]:+.2f}, {self.basket[1]:+.2f}) no map, '
            f'raio {self.basket_r:.2f} m')

    # --------------------------------------------------------------- bananas
    def remove_leftovers(self):
        restos = []
        while True:
            h = self.sim.getObject('/' + ALIAS, {'index': len(restos), 'noError': True})
            if h < 0:
                break
            restos.append(h)
        for h in restos:
            self.sim.removeObjects(self.sim.getObjectsInTree(h))
        if restos:
            self.get_logger().info(f'tirei {len(restos)} bananas de uma execução anterior')

    def spawn(self, floor, count, clearance):
        cx, cy = self.basket
        fixas = list(self.get_parameter('positions').value)
        if len(fixas) >= 2:
            pontos = list(zip(fixas[0::2], fixas[1::2]))
        else:
            pontos = [p for p in sample_spots(floor, count * 4, clearance=clearance,
                                              base_clearance=0.8, spacing=0.5, rng=self.rng)
                      if math.hypot(p[0] - cx, p[1] - cy) >= self.basket_r + 0.8][:count]
        sim = self.sim
        for x, y in pontos:
            box = sim.createPrimitiveShape(sim.primitiveshape_cuboid, BOX, 0)
            sim.setObjectAlias(box, ALIAS)
            sim.setShapeMass(box, MASS)
            sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)
            sim.setObjectInt32Param(box, sim.shapeintparam_respondable, 1)
            sim.setObjectInt32Param(box, sim.shapeintparam_respondable_mask, BANANA_BIT)
            sim.setObjectInt32Param(box, sim.objintparam_visibility_layer, 0)
            sim.setFloatArrayProperty(box, 'customData.graspOffset', GRASP_OFFSET)
            visual = sim.loadModel(self.model)
            sim.setObjectInt32Param(visual, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(visual, sim.shapeintparam_respondable, 0)
            sim.setObjectSpecialProperty(visual, sim.objectspecialproperty_renderable)
            sim.setObjectParent(visual, box, True)
            sim.setObjectPosition(visual, box, [0.0, 0.0, 0.0])
            sim.setObjectOrientation(visual, box, [0.0, 0.0, 0.0])
            sim.setObjectPosition(box, -1, [x + self.ox, y + self.oy, BOX[2] / 2 + 0.002])
            sim.setObjectOrientation(box, -1, [0.0, 0.0, self.rng.uniform(-math.pi, math.pi)])
            sim.resetDynamicObject(box)
            self.handles.append(box)
        self.get_logger().info(
            f'{len(pontos)} bananas no chão: '
            + ', '.join(f'({x:+.2f}, {y:+.2f})' for x, y in pontos))

    def tick(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        no_cesto = 0
        for _, m, parent in self.sim.callScriptFunction('bananas', self.iface):
            # Meio da banana no mundo, e o ângulo do comprimento dela (eixo x).
            gx = m[0] * GRASP_OFFSET[0] + m[1] * GRASP_OFFSET[1] + m[2] * GRASP_OFFSET[2] + m[3]
            gy = m[4] * GRASP_OFFSET[0] + m[5] * GRASP_OFFSET[1] + m[6] * GRASP_OFFSET[2] + m[7]
            gz = m[8] * GRASP_OFFSET[0] + m[9] * GRASP_OFFSET[1] + m[10] * GRASP_OFFSET[2] + m[11]
            x, y = m[3] - self.ox, m[7] - self.oy
            # Dentro do círculo do cesto, em qualquer altura: as bananas se
            # empilham acima da borda. Nenhuma é sorteada ali perto.
            if math.hypot(x - self.basket[0], y - self.basket[1]) < self.basket_r:
                no_cesto += 1
                continue
            if parent != -1:          # na garra
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = gx - self.ox, gy - self.oy, gz
            pose.orientation.z, pose.orientation.w = yaw_quaternion(math.atan2(m[4], m[0]))
            msg.poses.append(pose)
        if self.truth:
            self.bananas_pub.publish(msg)
        if no_cesto != self.delivered:
            if self.delivered >= 0 and no_cesto > self.delivered:
                self.get_logger().info(f'banana no cesto: {no_cesto}/{len(self.handles)}')
            self.delivered = no_cesto
            self.delivered_pub.publish(Int32(data=no_cesto))

    def clear(self):
        sim = RemoteAPIClient(port=self.port).getObject('sim')   # conexão nova (Ctrl+C)
        for h in self.handles:
            sim.removeObjects(sim.getObjectsInTree(h))
        self.handles = []


def main():
    rclpy.init()
    node = World()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.clear()
        except Exception as erro:
            node.get_logger().warn(f'não consegui tirar as bananas da cena: {erro}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
