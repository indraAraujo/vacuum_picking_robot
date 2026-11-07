#!/usr/bin/env python3
"""Teleoperação pelo teclado: a base e o braço no mesmo terminal.

    ros2 run picking_robot teleop

Precisa de um terminal de verdade (lê as teclas direto, sem Enter).
"""
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from . import arm

AJUDA = """
Base                      Braço (junta: + / -)
  i       frente            1 / q   junta 0 (gira a base do braço)
j k l     girar / parar     2 / w   junta 1 (ombro)
  ,       ré                3 / e   junta 2 (cotovelo)
                            4 / r   junta 3 (punho)
                            5 / t   junta 4 (gira a garra)
Garra                     Poses prontas
  espaço  abrir / fechar    z repouso   x chão   c carregar   v alto

Ctrl+C sai.
"""

BASE = {'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1), 'k': (0, 0)}
JUNTAS = {'1': (0, 1), 'q': (0, -1), '2': (1, 1), 'w': (1, -1), '3': (2, 1), 'e': (2, -1),
          '4': (3, 1), 'r': (3, -1), '5': (4, 1), 't': (4, -1)}
POSES = {'z': 'repouso', 'x': 'chao', 'c': 'carregar', 'v': 'alto'}


class Teleop(Node):

    def __init__(self):
        super().__init__('picking_teleop')
        self.declare_parameter('linear', 0.25)     # [m/s]
        self.declare_parameter('angular', 1.0)     # [rad/s]
        self.declare_parameter('step', 0.1)        # passo das juntas [rad]
        self.cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', 10)
        self.arm = self.create_publisher(Float64MultiArray, '/myRobot/arm/command', 10)
        self.gripper = self.create_publisher(Bool, '/myRobot/gripper/command', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.targets = None         # alvo atual do braço; começa onde ele está
        self.closed = False
        self.base = (0, 0)
        # Repete a velocidade: a ponte para as rodas se o comando parar de vir.
        self.create_timer(0.1, self.send_base)

    def on_joints(self, msg):
        if self.targets is None and len(msg.position) >= 5:
            self.targets = list(msg.position[:5])

    def send_base(self):
        t = Twist()
        t.linear.x = self.base[0] * self.get_parameter('linear').value
        t.angular.z = self.base[1] * self.get_parameter('angular').value
        self.cmd.publish(t)

    def send_arm(self):
        self.targets = arm.clamp(self.targets)
        self.arm.publish(Float64MultiArray(data=self.targets))

    def key(self, k):
        if k in BASE:
            self.base = BASE[k]
        elif k in JUNTAS:
            if self.targets is None:
                return 'esperando /joint_states (a ponte está rodando?)'
            j, sinal = JUNTAS[k]
            self.targets[j] += sinal * self.get_parameter('step').value
            self.send_arm()
            return f'junta {j}: {self.targets[j]:+.2f} rad'
        elif k in POSES:
            self.targets = list(arm.POSES[POSES[k]])
            self.send_arm()
            return f'pose {POSES[k]}'
        elif k == ' ':
            self.closed = not self.closed
            self.gripper.publish(Bool(data=self.closed))
            return 'garra ' + ('fechando' if self.closed else 'abrindo')
        return None


def main():
    if not sys.stdin.isatty():
        raise SystemExit('o teleop precisa de um terminal (rode com ros2 run, não pelo launch)')
    rclpy.init()
    node = Teleop()
    antigo = termios.tcgetattr(sys.stdin)
    print(AJUDA)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                k = sys.stdin.read(1)
                if k == '\x03':
                    break
                texto = node.key(k)
                if texto:
                    print(texto)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, antigo)
        # Para a base. No Ctrl+C o contexto do ROS já pode ter fechado; aí a
        # ponte para as rodas sozinha quando os comandos param de chegar.
        if rclpy.ok():
            node.base = (0, 0)
            node.send_base()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
