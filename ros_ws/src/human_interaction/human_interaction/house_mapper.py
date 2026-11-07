"""Mapeia a casa inteira sozinho: visita cada cômodo, gira e salva o mapa.

Roda com o projeto no ar (SLAM e Nav2) e com o laser **ligado**. Para cada
cômodo de rooms.py, manda o robô ao destino pelo Nav2 e dá uma volta completa
no lugar, porque o laser só vê 180 graus à frente. No fim, salva:

- o mapa (``<output>.pgm`` e ``<output>.yaml``), pelo map_saver;
- o estado do SLAM (``<output>_slam.data`` e ``.posegraph``), para continuar o
  mapeamento depois.

    ros2 run human_interaction house_mapper --ros-args \\
        -p output:=$PWD/ros_ws/src/vacuum_cleaner/maps/casa -p use_sim_time:=true

O percurso começa e termina na base. Cômodo que falha é anotado e o percurso
segue; a lista sai no fim.
"""

import math
import os
import subprocess

from nav2_msgs.action import NavigateToPose, Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph
import tf2_ros

from human_interaction import rooms


class HouseMapper(Node):

    def __init__(self):
        super().__init__('house_mapper')
        self.declare_parameter('output', '')
        self.declare_parameter('spin', True)
        self.declare_parameter('goal_timeout', 600.0)   # [s de relógio]
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_ac = ActionClient(self, Spin, '/spin')
        self.serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def run_action(self, client, goal, timeout):
        handle = self.wait(client.send_goal_async(goal), 30.0)
        if handle is None or not handle.accepted:
            return False
        result = self.wait(handle.get_result_async(), timeout)
        if result is None:
            handle.cancel_goal_async()
            return False
        return result.status == 4    # SUCCEEDED

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def go(self, name, xy):
        robot = self.robot_xy()
        yaw = rooms.heading(robot, xy) if robot else 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'indo para {name} ({xy[0]:+.2f}, {xy[1]:+.2f})')
        ok = self.run_action(self.nav, goal,
                             self.get_parameter('goal_timeout').value)
        if ok and self.get_parameter('spin').value:
            spin = Spin.Goal()
            spin.target_yaw = 2.0 * math.pi
            spin.time_allowance.sec = 120
            self.get_logger().info(f'girando em {name}')
            self.run_action(self.spin_ac, spin, 300.0)
        return ok

    def save(self, output):
        os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', output,
               '--ros-args', '-p', 'use_sim_time:=true',
               '-p', 'save_map_timeout:=30.0']
        saved = subprocess.run(cmd, capture_output=True, text=True).returncode == 0
        self.get_logger().info(
            f'mapa salvo em {output}.pgm/.yaml' if saved else 'falha ao salvar o mapa')
        if self.serialize.wait_for_service(timeout_sec=10.0):
            req = SerializePoseGraph.Request(filename=output + '_slam')
            self.wait(self.serialize.call_async(req), 60.0)
            self.get_logger().info(f'estado do SLAM salvo em {output}_slam.*')
        return saved

    def main(self):
        output = self.get_parameter('output').value
        if not output:
            self.get_logger().error('passe -p output:=<caminho sem extensão>')
            return 1
        for client, nome in ((self.nav, 'navigate_to_pose'), (self.spin_ac, 'spin')):
            if not client.wait_for_server(timeout_sec=60.0):
                self.get_logger().error(f'ação /{nome} não apareceu: o Nav2 está no ar?')
                return 1
        roteiro = [(r.name, r.goal) for r in rooms.ROOMS] + [('base', rooms.BASE)]
        falhas = [nome for nome, xy in roteiro if not self.go(nome, xy)]
        if falhas:
            self.get_logger().warn('não cheguei em: ' + ', '.join(falhas))
        return 0 if self.save(output) else 1


def main(args=None):
    rclpy.init(args=args)
    node = HouseMapper()
    try:
        code = node.main()
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
