#!/usr/bin/env python3
"""picker: recolhe as bananas do chão e as leva ao cesto.

Para cada banana, da mais próxima à mais distante:

1. **Ir até ela:** o Nav2 leva o robô a um ponto a ``approach_radius`` dela,
   virado para ela, por um lado com caminho livre até ela (plan.py).
2. **Encostar:** um controle próprio gira o robô para a banana e anda em linha
   reta até ela ficar a ``grasp_distance`` à frente do centro dele. Ele usa a
   posição mais recente da banana em ``/bananas``.
3. **Pegar:** gira o punho para a garra ficar atravessada na banana, desce a
   garra aberta (passando por cima dela), fecha e ergue. Confirma pela
   ``/myRobot/gripper/holding``.
4. **Entregar:** o Nav2 leva o robô para perto do cesto, o controle próprio
   encosta, o braço estende a banana sobre o cesto e a garra abre. A banana
   cai. O robô volta de ré pelo mesmo corredor até a distância de onde o
   Nav2 o trouxe, e passa para a próxima. Depois de pegar, ele faz o mesmo:
   assim o Nav2 sempre parte de um ponto livre, e não de perto do cesto ou de
   uma parede.

O braço viaja erguido (pose ``alto``), que é a pose mais compacta. Quem diz
onde as bananas estão é ``/bananas`` (geometry_msgs/PoseArray, frame map):
hoje o nó world, com a posição exata da simulação; depois, um nó de visão. O
picker não sabe a diferença. Uma banana que caiu fora do cesto volta a
aparecer em ``/bananas`` e é recolhida de novo.

Estado em ``/picking/state`` (std_msgs/String). ``/picking/start`` e
``/picking/stop`` (std_srvs/Trigger) começam e interrompem a tarefa; com
``autostart`` (padrão), ela começa sozinha.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
import tf2_ros
from vacuum_cleaner.debris import FloorMap

from . import arm
from .plan import approach_points, in_robot, nearest, wrap, wrist_for

#: Altura do centro do robô acima do chão [m] (origem do referencial do braço).
ROBOT_Z = -arm.FLOOR_Z


class Picker(Node):

    def __init__(self):
        super().__init__('picker')
        vc = get_package_share_directory('vacuum_cleaner')
        decl = self.declare_parameter
        decl('autostart', True)
        decl('map', os.path.join(vc, 'maps', 'casa.yaml'))
        decl('approach_radius', 0.65)   # Nav2 deixa o robô a isto da banana [m]
        decl('grasp_distance', 0.27)    # banana à frente do centro do robô ao pegar [m]
        decl('grasp_height', 0.010)     # ponta dos dedos acima do chão ao pegar [m]
        decl('basket_radius', 0.22)     # raio do cesto [m]
        decl('basket_approach', 0.90)   # Nav2 deixa o robô a isto do centro do cesto [m]
        decl('basket_distance', 0.42)   # centro do cesto à frente do robô ao soltar [m]
        decl('drop_height', 0.24)       # ponta dos dedos acima do chão ao soltar [m]
        decl('max_attempts', 2)         # tentativas por banana antes de desistir
        p = lambda n: self.get_parameter(n).value
        self.floor = FloorMap.from_yaml(p('map'))

        self.bananas = []          # [(x, y, z, yaw)] no map
        self.basket = None         # (x, y) no map
        self.joints = None
        self.holding = False
        self.delivered = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, '/bananas', self.on_bananas, 10)
        self.create_subscription(PoseStamped, '/basket', self.on_basket, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Bool, '/myRobot/gripper/holding',
                                 lambda m: setattr(self, 'holding', m.data), 10)
        self.create_subscription(Int32, '/bananas/delivered',
                                 lambda m: setattr(self, 'delivered', m.data), latched)
        self.cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/myRobot/arm/command', 10)
        self.grip_pub = self.create_publisher(Bool, '/myRobot/gripper/command', 10)
        self.state_pub = self.create_publisher(String, '/picking/state', latched)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # A ação existe antes de o Nav2 ficar ativo, e até lá ele recusa tudo.
        self.nav_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.create_service(Trigger, '/picking/start', self.on_start)
        self.create_service(Trigger, '/picking/stop', self.on_stop)

        self.task = None
        self.goal_handle = None
        self.set_state('parado')
        if p('autostart'):
            self.task = self.mission()
        self.create_timer(0.05, self.tick)      # 20 Hz no tempo da simulação

    # ------------------------------------------------------------- entradas
    def on_bananas(self, msg):
        self.bananas = [(q.position.x, q.position.y, q.position.z,
                         2.0 * math.atan2(q.orientation.z, q.orientation.w)) for q in msg.poses]

    def on_basket(self, msg):
        self.basket = (msg.pose.position.x, msg.pose.position.y)

    def on_joints(self, msg):
        self.joints = list(msg.position[:5])

    def on_start(self, _req, resp):
        if self.task is None:
            self.task = self.mission()
            resp.message = 'começando'
        else:
            resp.message = 'já estou trabalhando'
        resp.success = True
        return resp

    def on_stop(self, _req, resp):
        self.task = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.drive(0.0, 0.0)
        self.set_state('parado')
        resp.success = True
        resp.message = 'parei'
        return resp

    # --------------------------------------------------------------- básicos
    def tick(self):
        if self.task is None:
            return
        try:
            next(self.task)
        except StopIteration:
            self.task = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, texto):
        self.state_pub.publish(String(data=texto))
        self.get_logger().info(texto)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                2.0 * math.atan2(q.z, q.w))

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd.publish(msg)

    def clearance(self, x, y):
        return self.floor.clearance(x, y)

    # ------------------------------------------------------- passos (geradores)
    def wait(self, seconds):
        fim = self.now() + seconds
        while self.now() < fim:
            yield

    def arm_to(self, q, timeout=10.0):
        """Manda o braço para ``q`` e espera ele chegar."""
        q = arm.clamp(q)
        self.arm_pub.publish(Float64MultiArray(data=q))
        fim = self.now() + timeout
        while self.now() < fim:
            if self.joints and max(abs(a - b) for a, b in zip(self.joints, q)) < 0.02:
                return True
            yield
        self.get_logger().warn('o braço não chegou à pose a tempo')
        return False

    def gripper(self, close):
        self.grip_pub.publish(Bool(data=close))
        yield from self.wait(1.5 if close else 1.0)

    def navigate(self, x, y, yaw):
        """Nav2 até (x, y, yaw); devolve se chegou."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        enviado = self.nav.send_goal_async(goal)
        while not enviado.done():
            yield
        handle = enviado.result()
        if not handle.accepted:
            self.get_logger().warn('o Nav2 recusou o objetivo')
            return False
        self.goal_handle = handle
        resultado = handle.get_result_async()
        while not resultado.done():
            yield
        self.goal_handle = None
        return resultado.result().status == 4          # SUCCEEDED

    def go_near(self, target, radius, stop, nome):
        """Nav2 até um ponto de onde se chega a ``target`` em linha reta."""
        pose = self.pose()
        avoid = [(*self.basket, self.get_parameter('basket_radius').value)] if self.basket else []
        pontos = approach_points(target, radius, stop, self.clearance, pose[:2], avoid=avoid)
        if not pontos:
            self.get_logger().warn(f'nenhum lado livre para chegar {nome}')
            return False
        for x, y, yaw in pontos[:4]:
            yield from self.leave_basket()
            ok = yield from self.navigate(x, y, yaw)
            if ok:
                return True
            self.get_logger().warn(f'o Nav2 não chegou ao ponto ({x:+.2f}, {y:+.2f}); tento outro lado')
        return False

    def approach(self, target_fn, distance, timeout=40.0):
        """Gira para o alvo e anda reto até ele ficar a ``distance`` à frente."""
        fim = self.now() + timeout
        parado = 0
        while self.now() < fim:
            pose, alvo = self.pose(), target_fn()
            if pose is None or alvo is None:
                self.drive(0.0, 0.0)
                yield
                continue
            frente, lado = in_robot(pose, alvo)
            dist = math.hypot(frente, lado)
            rumo = math.atan2(lado, frente)
            erro = dist - distance
            if abs(rumo) > 0.06:
                v, w = 0.0, max(-0.6, min(0.6, 2.0 * rumo))
            else:
                v = max(-0.08, min(0.08, 0.8 * erro))
                w = max(-0.3, min(0.3, 2.0 * rumo))
            if abs(erro) < 0.008 and abs(rumo) < 0.02:
                v = w = 0.0
                parado += 1
                if parado >= 5:
                    self.drive(0.0, 0.0)
                    return True
            else:
                parado = 0
            self.drive(v, w)
            yield
        self.drive(0.0, 0.0)
        self.get_logger().warn('não consegui encostar no alvo a tempo')
        return False

    def retreat(self, target, distance, timeout=40.0):
        """De ré, virado para ``target``, até ficar a ``distance`` dele.

        Usado depois de pegar e de soltar: o robô volta pelo mesmo corredor
        por onde chegou, até a distância de onde o Nav2 o tinha trazido. Se o
        Nav2 partisse de perto do cesto (dentro da zona de custo dele), o
        controlador podia contorná-lo rente demais, entrar na zona proibida e
        travar ali.
        """
        fim = self.now() + timeout
        while self.now() < fim:
            pose = self.pose()
            if pose is None:
                yield
                continue
            frente, lado = in_robot(pose, target)
            if math.hypot(frente, lado) >= distance:
                break
            rumo = math.atan2(lado, frente)
            if abs(rumo) > 0.3:          # de costas para o alvo: vira antes
                self.drive(0.0, max(-0.6, min(0.6, 2.0 * rumo)))
            else:
                self.drive(-0.12, max(-0.3, min(0.3, 2.0 * rumo)))
            yield
        self.drive(0.0, 0.0)

    def leave_basket(self):
        """Se o robô estiver na zona do cesto, sai dela pelo lado em que está."""
        pose = self.pose()
        r = self.get_parameter('basket_radius').value
        if pose and self.basket and \
                math.hypot(pose[0] - self.basket[0], pose[1] - self.basket[1]) < r + 0.55:
            self.get_logger().info('perto demais do cesto: afastando antes de navegar')
            yield from self.retreat(self.basket, self.get_parameter('basket_approach').value)

    # --------------------------------------------------------------- tarefa
    def fetch(self, banana):
        """Vai até a banana e a pega. Devolve se está segurando."""
        alvo = [banana[:2]]

        def rastreia():
            # A banana mais perto de onde ela estava (na visão, ela "anda").
            b = nearest(self.bananas, alvo[0], max_dist=0.25)
            if b is not None:
                alvo[0] = b[:2]
                rastreia.ultima = b
            return alvo[0]
        rastreia.ultima = banana

        p = self.get_parameter
        ok = yield from self.go_near(banana[:2], p('approach_radius').value,
                                     p('grasp_distance').value, 'à banana')
        if not ok:
            return False
        ok = yield from self.approach(rastreia, p('grasp_distance').value)
        if not ok:
            return False
        b = rastreia.ultima
        pose = self.pose()
        frente, _ = in_robot(pose, b[:2])
        punho = wrist_for(b[3], pose[2])
        z = p('grasp_height').value - ROBOT_Z
        acima = arm.reach(frente, z + 0.10, wrist=punho)
        pega = arm.reach(frente, z, wrist=punho)
        if acima is None or pega is None:
            self.get_logger().warn(f'a banana está fora do alcance do braço ({frente:.2f} m)')
            return False
        yield from self.gripper(False)
        yield from self.arm_to(acima)
        yield from self.arm_to(pega)
        yield from self.gripper(True)
        segurando = self.holding
        yield from self.arm_to(acima)
        yield from self.arm_to(arm.POSES['alto'])
        yield from self.retreat(b[:2], p('approach_radius').value)
        return segurando and self.holding

    def deliver(self):
        """Leva a banana segura até o cesto e a solta lá dentro."""
        p = self.get_parameter
        ok = yield from self.go_near(self.basket, p('basket_approach').value,
                                     p('basket_distance').value, 'ao cesto')
        if not ok:
            return False
        ok = yield from self.approach(lambda: self.basket, p('basket_distance').value)
        if not ok:
            return False
        solta = arm.reach_any(p('basket_distance').value - 0.02,
                              p('drop_height').value - ROBOT_Z, wrist=math.pi / 2)
        yield from self.arm_to(solta)
        yield from self.gripper(False)
        yield from self.wait(1.0)
        yield from self.arm_to(arm.POSES['alto'])
        yield from self.retreat(self.basket, p('basket_approach').value)
        return True

    def nav_active(self):
        """Espera o bt_navigator ficar ativo."""
        while True:
            if self.nav_state.service_is_ready():
                f = self.nav_state.call_async(GetState.Request())
                while not f.done():
                    yield
                if f.result() and f.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return
            yield from self.wait(1.0)

    def mission(self):
        self.set_state('esperando o Nav2, a pose do robô, o cesto e as bananas')
        while not (self.nav.server_is_ready() and self.pose() and self.basket
                   and self.joints is not None):
            yield
        yield from self.nav_active()
        yield from self.wait(2.0)                  # as bananas assentarem no chão
        yield from self.gripper(False)
        yield from self.arm_to(arm.POSES['alto'])
        falhas = []                                # [(x, y, tentativas)]
        vazio_desde = None
        while True:
            pose = self.pose()
            candidatas = [b for b in self.bananas
                          if not any(math.hypot(b[0] - x, b[1] - y) < 0.15 and n >= self.get_parameter('max_attempts').value
                                     for x, y, n in falhas)]
            if not candidatas:
                vazio_desde = vazio_desde or self.now()
                if self.now() - vazio_desde > 5.0:
                    break
                yield
                continue
            vazio_desde = None
            banana = min(candidatas, key=lambda b: math.hypot(b[0] - pose[0], b[1] - pose[1]))
            self.set_state(f'indo pegar a banana em ({banana[0]:+.2f}, {banana[1]:+.2f})')
            pegou = yield from self.fetch(banana)
            if not pegou:
                yield from self.gripper(False)
                yield from self.arm_to(arm.POSES['alto'])
                registro = nearest(falhas, banana[:2], max_dist=0.15)
                if registro:
                    falhas.remove(registro)
                    falhas.append((banana[0], banana[1], registro[2] + 1))
                else:
                    falhas.append((banana[0], banana[1], 1))
                self.set_state('não consegui pegar essa banana; vou para a próxima')
                continue
            self.set_state('banana na garra: levando ao cesto')
            entregou = yield from self.deliver()
            if not entregou:
                self.set_state('não consegui chegar ao cesto; soltando a banana aqui')
                yield from self.gripper(False)
                yield from self.arm_to(arm.POSES['alto'])
            yield from self.wait(1.0)
        restantes = len(self.bananas)
        yield from self.arm_to(arm.POSES['repouso'])
        self.set_state(f'terminei: {self.delivered} bananas no cesto'
                       + (f', {restantes} não consegui pegar' if restantes else ''))


def main():
    rclpy.init()
    node = Picker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
