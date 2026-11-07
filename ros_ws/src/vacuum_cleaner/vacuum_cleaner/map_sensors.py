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

A área de limpeza (Area) entra do mesmo jeito: fora dela é parede. É o que
prende a varredura ao cômodo pedido, fechando as portas e as passagens abertas
com uma parede virtual. Funciona também com o lidar (area_readings).
"""

import math

from .coverage import BEAM_HALF_WIDTH, SENSOR_RANGE, SENSORS

#: Valor a partir do qual uma célula do OccupancyGrid é obstáculo.
OCCUPIED = 50
#: Passo dos raios só na área (sem mapa) [m].
AREA_STEP = 0.025


def cast(blocked, x, y, theta, step):
    """Leituras dos cinco sensores com base_link em (x, y, theta).

    ``blocked(x, y)`` diz se o ponto é obstáculo. Cada sensor vira três raios
    paralelos (o centro e as bordas do feixe); vale o mais curto.
    """
    c, s = math.cos(theta), math.sin(theta)
    leituras = {}
    for nome, ((sx, sy), (bx, by), offset) in SENSORS.items():
        ox, oy = x + sx * c - sy * s, y + sx * s + sy * c
        dx, dy = bx * c - by * s, bx * s + by * c
        nx, ny = -dy, dx              # normal ao feixe
        melhor = math.inf
        for w in (-BEAM_HALF_WIDTH, 0.0, BEAM_HALF_WIDTH):
            px, py = ox + w * nx, oy + w * ny
            d = offset
            while d <= min(SENSOR_RANGE, melhor):
                if blocked(px + d * dx, py + d * dy):
                    melhor = d
                    break
                d += step
        leituras[nome] = melhor
    return leituras


class Area:
    """União de retângulos (x_min, x_max, y_min, y_max) no frame map.

    ``margin`` alarga todos os retângulos: o cleaning_server a usa quando o
    robô está fora da área (por exemplo, se a limpeza começou na porta), para que a
    parede virtual não fique em cima dele.
    """

    def __init__(self, rects, margin=0.0):
        self.rects = [tuple(r) for r in rects]
        self.margin = margin

    @classmethod
    def from_flat(cls, data):
        """[x_min, x_max, y_min, y_max, ...] -> Area, ou None se vazio."""
        data = list(data)
        if not data:
            return None
        if len(data) % 4:
            raise ValueError('a área precisa de 4 números por retângulo')
        return cls([data[i:i + 4] for i in range(0, len(data), 4)])

    def contains(self, x, y):
        m = self.margin
        return any(x0 - m <= x <= x1 + m and y0 - m <= y <= y1 + m
                   for x0, x1, y0, y1 in self.rects)

    def follow(self, x, y, slack=0.10):
        """Ajusta a margem à posição do robô e devolve quanto ele está fora.

        Dentro da área, margem zero. Quando ele aparece fora (a limpeza começou
        na porta, por exemplo), a margem salta até alcançá-lo, com ``slack`` de
        folga. Depois disso ela só encolhe, acompanhando-o na volta: se ele se
        afastar, bate na parede virtual em vez de levá-la junto.
        """
        fora = self.distance_outside(x, y)
        if fora <= 0.0:
            self.margin = 0.0
        elif self.margin <= 0.0:
            self.margin = fora + slack
        else:
            self.margin = min(self.margin, fora + slack)
        return fora

    def distance_outside(self, x, y):
        """0 dentro da área (sem margem); senão, a distância até ela."""
        return min(math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
                   for x0, x1, y0, y1 in self.rects)


def area_readings(area, x, y, theta):
    """Leituras dos sensores vendo só a borda da área como parede."""
    return cast(lambda px, py: not area.contains(px, py), x, y, theta, AREA_STEP)


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

    def readings(self, x, y, theta, area=None):
        """Leituras dos cinco sensores com base_link em (x, y, theta) no mapa.

        Com ``area``, o que estiver fora dela também conta como parede.
        """
        if area is None:
            return cast(self.blocked, x, y, theta, self.step)
        return cast(lambda px, py: not area.contains(px, py) or self.blocked(px, py),
                    x, y, theta, self.step)
