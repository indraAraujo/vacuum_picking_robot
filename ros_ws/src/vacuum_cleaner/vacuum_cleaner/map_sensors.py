"""Sensores de proximidade virtuais lidos do mapa, sem o lidar.

O robô conhece o mapa da casa e a própria pose (a odometria vem da simulação,
sem erro). Então dá para saber a que distância está a parede sem ligar o
laser: cada sensor de coverage.SENSORS vira três raios paralelos (o centro e
as bordas do feixe) percorridos célula a célula no mapa. A leitura é a
distância, ao longo do feixe, da primeira célula ocupada.

Célula desconhecida (-1) ou fora do mapa conta como obstáculo: as paredes da
cena não têm colisão, então é melhor parar onde o mapa acaba do que seguir
sem saber o que há ali.

Só não vê o que não está no mapa: móveis que mudaram de lugar ou objetos
deixados no chão depois do mapeamento.
"""

import math

from .coverage import BEAM_HALF_WIDTH, SENSOR_RANGE, SENSORS

#: Valor a partir do qual uma célula do OccupancyGrid é obstáculo.
OCCUPIED = 50


class MapSensors:

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 origin_yaw=0.0):
        self.data = data
        self.width = width
        self.height = height
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y
        self.cos0 = math.cos(origin_yaw)
        self.sin0 = math.sin(origin_yaw)
        # Passo de meia célula: um raio não pula uma parede de uma célula.
        self.step = resolution / 2.0

    @classmethod
    def from_msg(cls, msg):
        """A partir de um nav_msgs/OccupancyGrid."""
        info = msg.info
        q = info.origin.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        return cls(msg.data, info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y, yaw)

    def blocked(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        gx = (dx * self.cos0 + dy * self.sin0) / self.res
        gy = (-dx * self.sin0 + dy * self.cos0) / self.res
        i, j = math.floor(gx), math.floor(gy)
        if not (0 <= i < self.width and 0 <= j < self.height):
            return True
        v = self.data[j * self.width + i]
        return v < 0 or v >= OCCUPIED

    def ray(self, x, y, dx, dy, start, stop):
        """Distância ao longo de (dx, dy) da primeira célula ocupada."""
        d = start
        while d <= stop:
            if self.blocked(x + d * dx, y + d * dy):
                return d
            d += self.step
        return math.inf

    def readings(self, x, y, theta):
        """Leituras dos cinco sensores com base_link em (x, y, theta) no mapa."""
        c, s = math.cos(theta), math.sin(theta)
        leituras = {}
        for nome, ((sx, sy), (bx, by), offset) in SENSORS.items():
            ox, oy = x + sx * c - sy * s, y + sx * s + sy * c
            dx, dy = bx * c - by * s, bx * s + by * c
            nx, ny = -dy, dx              # normal ao feixe
            leituras[nome] = min(
                self.ray(ox + w * nx, oy + w * ny, dx, dy, offset, SENSOR_RANGE)
                for w in (-BEAM_HALF_WIDTH, 0.0, BEAM_HALF_WIDTH))
        return leituras
