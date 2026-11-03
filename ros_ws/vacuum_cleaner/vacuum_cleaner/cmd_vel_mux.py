"""Multiplexador de comandos de velocidade com prioridade para a teleoperação.

Entradas:
    teleop_topic (padrão /cmd_vel_teleop): comandos do teclado, prioridade alta.
    nav_topic    (padrão /cmd_vel):        comandos do Nav2 (collision_monitor,
                                           docking_server), prioridade baixa.
Saída:
    output_topic (padrão /cmd_vel_mux):    tópico assinado pelo subscriber_velocity.

Enquanto houver comando de teleop há menos de `teleop_timeout` segundos,
os comandos do Nav2 são descartados.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        teleop_topic = self.declare_parameter('teleop_topic', '/cmd_vel_teleop').value
        nav_topic = self.declare_parameter('nav_topic', '/cmd_vel').value
        output_topic = self.declare_parameter('output_topic', '/cmd_vel_mux').value
        self.teleop_timeout = float(self.declare_parameter('teleop_timeout', 0.5).value)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, teleop_topic, self.teleop_callback, 10)
        self.create_subscription(Twist, nav_topic, self.nav_callback, 10)

        self.last_teleop_time = None
        self.get_logger().info(
            f'Mux: {teleop_topic} (prioridade) + {nav_topic} -> {output_topic}')

    def teleop_active(self):
        if self.last_teleop_time is None:
            return False
        elapsed = (self.get_clock().now() - self.last_teleop_time).nanoseconds * 1e-9
        return elapsed < self.teleop_timeout

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()
        self.publisher.publish(msg)

    def nav_callback(self, msg):
        if not self.teleop_active():
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
