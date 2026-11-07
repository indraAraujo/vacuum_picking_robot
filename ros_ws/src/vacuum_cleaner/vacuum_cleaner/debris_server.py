#!/usr/bin/env python3
"""
debris_server
-------------
Espalha bananas pelo chão da casa e as recolhe quando o robô passa por cima
limpando, como a sujeira dos projetos de referência (``slam_field`` e
``4 - roomba``).

- Ao subir, sorteia ``count`` posições no chão livre do mapa (ver debris.py) e
  carrega o modelo ``models/banana.ttm`` em cada uma, com um giro sorteado.
- Com o robô no modo CLEANING, a banana a até ``pickup_radius`` do centro do
  robô some da cena: foi aspirada. Fora da limpeza o robô passa por cima sem
  recolher (o aspirador está desligado).
- Ao sair, tira da cena as bananas que sobraram. As bananas são criadas pela
  Remote API e não ficam salvas na cena, a não ser que alguém salve a cena com
  elas.

As bananas não são obstáculo: ficam estáticas, sem colisão e invisíveis para
o laser e para os sensores, e o robô passa por cima delas.

Tópicos e serviços:

- ``/myRobot/debris`` (geometry_msgs/PoseArray, frame map, latched): as
  bananas que ainda estão no chão;
- ``/myRobot/debris/respawn`` (std_srvs/Trigger): tira as que sobraram e
  espalha um lote novo.
"""
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .debris import FloorMap, sample_spots
from .modes import Mode

#: Alias das bananas na cena, para achar as que sobraram de uma execução anterior.
ALIAS = 'debrisBanana'


class DebrisServer(Node):

    def __init__(self):
        super().__init__('debris_server')
        share = get_package_share_directory('vacuum_cleaner')
        self.declare_parameter('sim_port', 23000)
        self.declare_parameter('map', os.path.join(share, 'maps', 'casa.yaml'))
        self.declare_parameter('model', os.path.join(share, 'models', 'banana.ttm'))
        self.declare_parameter('count', 15)
        self.declare_parameter('seed', -1)               # -1: sorteio diferente a cada vez
        self.declare_parameter('pickup_radius', 0.15)    # como o /Dirt do 4 - roomba
        self.declare_parameter('clearance', 0.30)        # das paredes [m]
        # Posição do frame map na cena (a pose inicial do robô; ver rooms.py).
        self.declare_parameter('scene_origin', [3.55, 0.775])
        p = lambda n: self.get_parameter(n).value
        self.ox, self.oy = p('scene_origin')
        self.pickup = float(p('pickup_radius'))
        seed = int(p('seed'))
        self.rng = random.Random(None if seed < 0 else seed)

        self.sim_port = int(p('sim_port'))
        self.sim = RemoteAPIClient(port=self.sim_port).getObject('sim')
        self.robot = self.sim.getObject('/myRobot')
        with open(p('model'), 'rb') as f:
            # O conteúdo, não o caminho: o CoppeliaSim lê qualquer caminho, mas
            # assim o nó não depende de onde o simulador está rodando.
            self.model = f.read()
        self.floor = FloorMap.from_yaml(p('map'))
        self.bananas = {}        # handle -> (x, y) no frame map
        self.total = 0
        self.mode = Mode.IDLE

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseArray, '/myRobot/debris', latched)
        self.create_subscription(String, '/myRobot/behaviour/mode', self.on_mode,
                                 QoSProfile(depth=1))
        self.create_service(Trigger, '/myRobot/debris/respawn', self.on_respawn)

        self.remove_leftovers()
        self.spawn()
        self.create_timer(0.05, self.check_pickup)

    # ----------------------------------------------------------------- cena
    def remove_leftovers(self):
        restos = []
        while True:
            h = self.sim.getObject('/' + ALIAS, {'index': len(restos), 'noError': True})
            if h < 0:
                break
            restos.append(h)
        if restos:
            self.sim.removeObjects(restos)
            self.get_logger().info(f'tirei {len(restos)} bananas de uma execução anterior')

    def spawn(self):
        pontos = sample_spots(self.floor, int(self.get_parameter('count').value),
                              clearance=float(self.get_parameter('clearance').value),
                              rng=self.rng)
        for x, y in pontos:
            h = self.sim.loadModel(self.model)
            self.sim.setObjectAlias(h, ALIAS)
            # Só aparece: sem colisão, sem detecção pelo laser e pelos sensores.
            self.sim.setObjectSpecialProperty(h, self.sim.objectspecialproperty_renderable)
            z = self.sim.getObjectPosition(h, -1)[2]
            self.sim.setObjectPosition(h, -1, [x + self.ox, y + self.oy, z])
            a, b, _ = self.sim.getObjectOrientation(h, -1)
            self.sim.setObjectOrientation(h, -1, [a, b, self.rng.uniform(-math.pi, math.pi)])
            self.bananas[h] = (x, y)
        self.total = len(self.bananas)
        self.get_logger().info(
            f'{self.total} bananas espalhadas pelo chão: '
            + ', '.join(f'({x:+.2f}, {y:+.2f})' for x, y in pontos))
        self.publish()

    def clear(self):
        if self.bananas:
            self.sim.removeObjects(list(self.bananas))
        self.bananas = {}

    def publish(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.bananas.values():
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)

    # ------------------------------------------------------------- entradas
    def on_mode(self, msg):
        try:
            self.mode = Mode(msg.data)
        except ValueError:
            self.mode = Mode.IDLE

    def on_respawn(self, _req, resp):
        self.clear()
        self.spawn()
        resp.success = True
        resp.message = f'{self.total} bananas novas'
        return resp

    def check_pickup(self):
        if self.mode != Mode.CLEANING or not self.bananas:
            return
        rx, ry, _ = self.sim.getObjectPosition(self.robot, -1)
        rx, ry = rx - self.ox, ry - self.oy
        pegas = [h for h, (x, y) in self.bananas.items()
                 if math.hypot(x - rx, y - ry) <= self.pickup]
        if not pegas:
            return
        self.sim.removeObjects(pegas)
        for h in pegas:
            x, y = self.bananas.pop(h)
            self.get_logger().info(
                f'banana aspirada em ({x:+.2f}, {y:+.2f}): '
                f'{self.total - len(self.bananas)}/{self.total}')
        if not self.bananas:
            self.get_logger().info('todas as bananas foram aspiradas')
        self.publish()


def main():
    rclpy.init()
    node = DebrisServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # Conexão nova: o Ctrl+C pode ter interrompido uma chamada no meio
            # e deixado o socket da antiga num estado inválido.
            node.sim = RemoteAPIClient(port=node.sim_port).getObject('sim')
            node.clear()
        except Exception as erro:  # o simulador pode já ter fechado
            node.get_logger().warn(f'não consegui tirar as bananas da cena: {erro}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
