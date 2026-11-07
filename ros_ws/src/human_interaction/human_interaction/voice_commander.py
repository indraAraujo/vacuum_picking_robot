r"""Comando de voz -> Nav2: "vá para a cozinha" vira um destino no mapa.

É aqui que a camada de voz encontra a navegação. O nó assina o comando que o
command_parser publica, descobre o ponto do cômodo pedido em rooms.py e manda
o destino para o Nav2 pela ação ``navigate_to_pose``. O planejamento, o desvio
de obstáculos e o controle das rodas ficam com o Nav2.

    voice/command (String, JSON) --->  |                 | ---> /navigate_to_pose (ação)
    TF map -> base_link          --->  | voice_commander | ---> house/task (String)
                                       |                 | ---> house/room (String)

Os comandos que movem o robô (mesmo comportamento do house_navigator do
pacote voice_cleaner):

    ir_para      local   vai até o cômodo
    limpar_local local   vai até o cômodo pelo Nav2 e começa a varredura lá
    limpar_tudo          começa a varredura onde está
    limpar_aqui          idem
    voltar_base          volta ao ponto onde o robô começou
    mapear               passa por todos os cômodos, um a um
    parar                cancela a fila, encerra a limpeza e para
    pausar / continuar   para sem perder a fila, e retoma de onde parou
    status               diz onde está, para onde vai e quanto falta

A varredura é a do cleaning_server (faixas paralelas, ver vacuum_cleaner/
coverage.py), ligada pelo serviço /myRobot/start_stop da máquina de estados.
Pedir "a casa toda" em limpar_local também começa a varredura onde está.
Durante a limpeza, comandos de deslocamento são recusados: diga "parar" antes.

O lidar no modo de voz
----------------------
O laser é o que mais pesa na simulação. Com ``lidar:=false`` (o padrão), este
nó desliga o laser na cena enquanto estiver rodando e o religa ao sair.

Duas coisas acompanham o desligamento:

- o SLAM Toolbox pausa as medições novas, e o mapa fica congelado como estava;
- o myrobot_server passa a publicar um ``/scan`` vazio (sem nenhum eco). Ele
  existe porque o SLAM carimba a TF ``map -> odom`` com o horário do último
  scan: sem scan nenhum, essa TF envelhece e o Nav2 perde a pose do robô. O
  scan vazio também mantém o collision_monitor satisfeito, que de outra forma
  pararia o robô por falta de dados.

Sem o laser, o Nav2 navega só pelo mapa que o SLAM já tinha montado: obstáculos
novos não são vistos.

A limpeza também roda com o laser desligado: o cleaning_server acha as
paredes no mapa, a partir da pose exata da simulação (vacuum_cleaner/
map_sensors.py).
"""

import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from slam_toolbox.srv import Pause
from std_srvs.srv import SetBool
from vacuum_cleaner_interfaces.srv import StartStop
import tf2_ros

from human_interaction import commands, rooms


class VoiceCommander(Node):
    """Leva o robô ao cômodo pedido por voz, usando o Nav2."""

    def __init__(self):
        super().__init__('voice_commander')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action', '/navigate_to_pose')
        # False desliga o laser enquanto o modo de voz estiver ativo; True
        # deixa como está.
        self.declare_parameter('lidar', False)
        self.declare_parameter('lidar_service', '/myRobot/lidar/enable')
        self.declare_parameter('slam', '/slam_toolbox')
        # Quanto esperar pelo SLAM antes de concluir que ele não está rodando
        # (relógio de parede, em segundos).
        self.declare_parameter('slam_wait', 10.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.queue = []            # [(nome, (x, y))] ainda por visitar
        self.goal_handle = None    # destino em andamento no Nav2
        self.goal_seq = 0          # invalida respostas de destinos cancelados
        self.paused = False
        self.remaining = None      # distância restante, do feedback do Nav2
        self.room = None           # cômodo onde o robô está

        self.nav = ActionClient(self, NavigateToPose,
                                self.get_parameter('action').value)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_pub = self.create_publisher(String, 'house/task', latched)
        self.room_pub = self.create_publisher(String, 'house/room', latched)
        self.create_subscription(String, 'voice/command',
                                 self.command_callback, 10)

        # Limpeza: a máquina de estados do behaviour_server decide o modo, e o
        # cleaning_server varre quando o modo é CLEANING.
        self.mode = None
        self.clean_when_done = False     # começar a varrer ao fim da fila
        self.start_stop = self.create_client(StartStop, '/myRobot/start_stop')
        self.create_subscription(String, '/myRobot/behaviour/mode',
                                 self.on_mode, QoSProfile(depth=1))
        self.create_timer(1.0, self.update_room)

        # Desligar o lidar: primeiro o SLAM pausa as medições (para o scan
        # vazio não apagar o mapa), depois o laser sai da cena.
        slam = self.get_parameter('slam').value
        self.lidar_cli = self.create_client(
            SetBool, self.get_parameter('lidar_service').value)
        self.pause_cli = self.create_client(Pause, slam + '/pause_new_measurements')
        self.slam_param_cli = self.create_client(
            GetParameters, slam + '/get_parameters')
        self.lidar_off = not self.get_parameter('lidar').value
        self.lidar_done = {'slam': False, 'lidar': False}
        self.paused_slam = False      # fui eu que pausei o SLAM?
        self.lidar_pending = False
        self.lidar_since = Clock().now()
        if self.lidar_off:
            self.lidar_timer = self.create_timer(1.0, self.switch_lidar_off)

    # =========================================================================
    #  Lidar
    # =========================================================================

    def slam_paused_request(self):
        return GetParameters.Request(names=['paused_new_measurements'])

    @staticmethod
    def slam_is_paused(future):
        valores = future.result().values
        return bool(valores and valores[0].bool_value)

    def switch_lidar_off(self):
        """Tenta até o SLAM e o myrobot_server responderem."""
        if self.lidar_pending:
            return
        if not self.lidar_done['slam']:
            if not (self.slam_param_cli.service_is_ready()
                    and self.pause_cli.service_is_ready()):
                espera = (Clock().now() - self.lidar_since).nanoseconds * 1e-9
                if espera < self.get_parameter('slam_wait').value:
                    return
                # Projeto com mapa pronto (slam:=false): não há o que pausar.
                self.get_logger().info('SLAM não está rodando: uso o mapa pronto')
                self.lidar_done['slam'] = True
                return
            self.lidar_pending = True
            self.slam_param_cli.call_async(self.slam_paused_request()) \
                .add_done_callback(self.on_slam_state)
        elif not self.lidar_done['lidar']:
            if not self.lidar_cli.service_is_ready():
                self.get_logger().info(
                    'esperando o myrobot_server para desligar o lidar',
                    throttle_duration_sec=10.0)
                return
            self.lidar_pending = True
            self.lidar_cli.call_async(SetBool.Request(data=False)) \
                .add_done_callback(self.on_lidar_off)

    def on_slam_state(self, future):
        if self.slam_is_paused(future):
            # Já estava pausado por outra pessoa: não é meu para despausar.
            self.lidar_pending = False
            self.lidar_done['slam'] = True
            return
        # O serviço alterna o estado; como estava rodando, alternar pausa.
        self.pause_cli.call_async(Pause.Request()).add_done_callback(
            self.on_slam_paused)

    def on_slam_paused(self, future):
        self.lidar_pending = False
        self.paused_slam = True
        self.lidar_done['slam'] = True
        self.get_logger().info('SLAM pausado: o mapa fica como está')

    def on_lidar_off(self, future):
        self.lidar_pending = False
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'não consegui desligar o lidar: {resp.message}')
            return
        self.lidar_done['lidar'] = True
        self.lidar_timer.cancel()
        self.get_logger().info(
            'lidar desligado para o modo de voz; o Nav2 navega pelo mapa '
            'já construído (religo ao sair)')

    def restore_lidar(self, timeout=2.0):
        """Religa o laser e depois retoma o SLAM, se fui eu que pausei."""
        if not self.lidar_off:
            return
        if self.lidar_done['lidar'] and self.lidar_cli.service_is_ready():
            f = self.lidar_cli.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
            self.get_logger().info('lidar religado')
        if self.paused_slam and self.slam_param_cli.service_is_ready():
            f = self.slam_param_cli.call_async(self.slam_paused_request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
            if f.done() and self.slam_is_paused(f):
                f = self.pause_cli.call_async(Pause.Request())
                rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
                self.get_logger().info('SLAM retomado')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def robot_xy(self):
        """Posição do robô no frame map, ou None se a TF ainda não chegou."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def update_room(self):
        """Publica o cômodo atual quando ele muda."""
        xy = self.robot_xy()
        if xy is None:
            return
        room = rooms.room_at(*xy) or 'fora dos cômodos conhecidos'
        if room != self.room:
            if self.room is not None:
                self.get_logger().info(f'{self.room} -> {room}')
            self.room = room
            self.room_pub.publish(String(data=room))

    def command_callback(self, msg: String):
        try:
            comando = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'comando ilegível: {msg.data!r}')
            return
        intent = comando.get('intent')
        local = (comando.get('args') or {}).get('local')
        self.dispatch(intent, local)

    # =========================================================================
    #  O que fazer com cada comando
    # =========================================================================

    def dispatch(self, intent, local):
        if intent == 'parar':
            self.stop('comando: parar')
        elif intent == 'pausar':
            self.pause()
        elif intent == 'continuar':
            self.resume()
        elif intent == 'status':
            self.report()
        elif intent == 'voltar_base':
            self.go([('base', rooms.BASE)], 'voltando para a base')
        elif intent == 'mapear':
            self.go(self.all_rooms(), 'percorrendo a casa')
        elif intent in ('limpar_tudo', 'limpar_aqui'):
            self.start_cleaning('limpando a partir daqui')
        elif intent in ('ir_para', 'limpar_local'):
            self.go_to_place(intent, local)
        elif intent == 'ajuda':
            for linha in commands.help_lines():
                self.get_logger().info(linha)
        elif intent == 'ajustar_velocidade':
            # Entendido, mas não muda nada. Dizer isso evita a dúvida "ele
            # ouviu e não fez nada?".
            self.get_logger().info(
                f'{intent}: entendi, mas ainda não ajusto a velocidade')

    @staticmethod
    def all_rooms():
        return [(room.name, room.goal) for room in rooms.ROOMS]

    def go_to_place(self, intent, local):
        if not local:
            self.get_logger().warn(
                f'{intent} sem lugar: diga "vá para a cozinha", por exemplo')
            return
        if local == 'casa toda':
            self.start_cleaning('limpando a casa a partir daqui')
            return
        room = rooms.BY_NAME.get(local)
        if room is None:
            self.get_logger().warn(
                f'"{local}" não é um cômodo desta casa. Tenho: '
                + ', '.join(rooms.room_names()))
            return
        limpar = intent == 'limpar_local'
        verbo = 'indo limpar' if limpar else 'indo para'
        self.go([(room.name, room.goal)], f'{verbo} {room.article} {room.name}',
                then_clean=limpar)

    def go(self, destinos, motivo, then_clean=False):
        """Troca a fila pelos destinos novos e começa pelo primeiro."""
        if self.mode == 'CLEANING':
            self.get_logger().warn('estou limpando: diga "parar" antes de me mandar para outro lugar')
            return
        if not self.nav.server_is_ready():
            self.get_logger().warn(
                'o Nav2 ainda não está pronto (/navigate_to_pose). O launch '
                'principal está rodando e com "Managed nodes are active"?')
            return
        self.cancel_goal()
        self.queue = list(destinos)
        self.paused = False
        self.clean_when_done = then_clean
        self.get_logger().info(motivo)
        self.send_next()

    def pause(self):
        if not self.queue:
            self.get_logger().info('pausar: não estou indo a lugar nenhum')
            return
        self.cancel_goal()
        self.paused = True
        self.announce(f'pausado a caminho de {self.queue[0][0]}')
        self.get_logger().info('pausado; diga "continuar" para retomar')

    def resume(self):
        if not self.paused:
            self.get_logger().info('continuar: não estava pausado')
            return
        self.paused = False
        self.get_logger().info(f'retomando para {self.queue[0][0]}')
        self.send_next()

    def stop(self, motivo):
        self.cancel_goal()
        self.queue = []
        self.paused = False
        self.clean_when_done = False
        if self.mode == 'CLEANING':
            self.call_start_stop('stop')
        self.announce('parado')
        self.get_logger().info(f'{motivo}: parei e esvaziei a fila')

    # =========================================================================
    #  Limpeza
    # =========================================================================

    def start_cleaning(self, motivo):
        if self.mode == 'CLEANING':
            self.get_logger().info('já estou limpando')
            return
        self.cancel_goal()
        self.queue = []
        self.paused = False
        # O laser continua desligado: o cleaning_server acha as paredes no
        # mapa, a partir da pose da simulação (map_sensors.py).
        if self.call_start_stop('start'):
            self.announce('limpando')
            self.get_logger().info(motivo)

    def call_start_stop(self, comando):
        if not self.start_stop.service_is_ready():
            self.get_logger().warn(
                'o serviço /myRobot/start_stop não está disponível: o '
                'behaviour_server está rodando?')
            return False
        self.start_stop.call_async(StartStop.Request(command=comando))
        return True

    def on_mode(self, msg: String):
        anterior, self.mode = self.mode, msg.data
        if anterior == 'CLEANING' and self.mode != 'CLEANING':
            self.get_logger().info(f'limpeza encerrada (modo {self.mode})')
            if not self.queue:
                self.announce('parado')

    def report(self):
        onde = self.room or '?'
        if not self.queue:
            self.get_logger().info(f'estou em: {onde}, sem destino')
            return
        estado = 'pausado' if self.paused else 'indo'
        falta = (f', faltam {self.remaining:.1f} m'
                 if self.remaining is not None and not self.paused else '')
        self.get_logger().info(
            f'{estado} para {self.queue[0][0]}, agora em: {onde}{falta}; '
            'fila: ' + ', '.join(nome for nome, _ in self.queue))

    # =========================================================================
    #  Nav2
    # =========================================================================

    def send_next(self):
        if not self.queue:
            if self.clean_when_done:
                self.clean_when_done = False
                self.start_cleaning('cheguei: começando a varredura')
                return
            self.announce('parado')
            self.get_logger().info('fila vazia: terminei')
            return
        nome, (x, y) = self.queue[0]
        robot = self.robot_xy()
        yaw = rooms.heading(robot, (x, y)) if robot else 0.0

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_seq += 1
        seq = self.goal_seq
        self.remaining = None
        self.announce(f'indo para {nome}')
        self.get_logger().info(f'destino {nome}: ({x:+.2f}, {y:+.2f}) no map')
        future = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(lambda f: self.on_goal_response(f, seq, nome))

    def on_goal_response(self, future, seq, nome):
        handle = future.result()
        if seq != self.goal_seq:
            # Já foi substituído por outro comando: cancela o que sobrou.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error(f'o Nav2 recusou o destino {nome}')
            self.skip(nome)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, seq, nome))

    def on_feedback(self, msg):
        self.remaining = msg.feedback.distance_remaining

    def on_result(self, future, seq, nome):
        if seq != self.goal_seq:
            return    # destino cancelado por pausa, parada ou comando novo
        self.goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'cheguei: {nome}')
            self.queue = self.queue[1:]
            self.send_next()
        else:
            erro = future.result().result.error_code
            self.get_logger().error(
                f'não consegui chegar em {nome} (status {status}, '
                f'erro {erro})')
            self.skip(nome)

    def skip(self, nome):
        self.queue = self.queue[1:]
        if self.queue:
            self.get_logger().info(f'pulando {nome}; vou ao próximo')
        self.send_next()

    def cancel_goal(self):
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def announce(self, texto):
        self.task_pub.publish(String(data=texto))


def main(args=None):
    # Sem o tratador de sinal do rclpy: no Ctrl+C o contexto continua vivo e
    # dá para religar o lidar antes de sair.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VoiceCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop('encerrando')
            node.restore_lidar()
        except Exception as erro:
            node.get_logger().warn(f'não consegui religar o lidar: {erro}')
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
