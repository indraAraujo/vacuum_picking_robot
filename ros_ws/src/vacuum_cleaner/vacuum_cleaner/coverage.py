"""Varredura em faixas paralelas (boustrophedon) para o robô aspirador.

Portado do controlador "2 - python_controller.py" do projeto 4 - roomba
(repositório coppeliasim-saves). A estratégia é a mesma:

  1. Anda reto até encontrar o primeiro obstáculo (em princípio, uma parede).
  2. Gira até ficar paralelo a essa parede e a segue lado a lado: é a
     primeira faixa. O ângulo da parede vem dos três sensores da frente.
  3. Ao encontrar um obstáculo, contorna-o sempre para o lado da varredura,
     isto é, para o lado ainda não limpo.
  4. Durante o contorno mede o afastamento perpendicular à linha da faixa.
     Quando ele chega a um diâmetro do robô, gira e começa a faixa seguinte,
     paralela à anterior e no sentido oposto (a curva em "U").
  5. Repete enquanto estiver no modo de limpeza.

E os mesmos tratamentos de exceção: obstáculo estreito (retoma a faixa depois
dele), fim da sala (inverte o lado da varredura) e armadilhas (corredor curto,
contorno longo demais ou encalhes repetidos: gira para o lado mais livre num
ângulo sorteado e recomeça do passo 1).

O que mudou em relação ao original
-----------------------------------
- Sensores: o original cria cópias do sensor de proximidade na cena. Aqui os
  mesmos cinco sensores são **virtuais**, calculados a partir do lidar 2D
  (função :func:`virtual_sensors`), nas mesmas posições e direções.
- Referencial: o original usa o referencial do objeto na cena (+y para a
  frente, +x para a direita). Aqui tudo está em ``base_link`` (+x para a
  frente, +y para a esquerda). As distâncias e limiares são os mesmos.
- Saída: em vez de escrever nos motores, :meth:`Coverage.step` devolve a
  velocidade linear e angular (o Twist que o myrobot_server converte para as
  rodas com a mesma cinemática do original).
- Posição: a pose do centro do eixo das rodas vem da odometria, que nesta
  simulação é a pose exata da cena (o equivalente ao ``ground_truth``).

Esta classe não depende de ROS, para poder ser testada sozinha.
"""

import math
import random

# ----------------------------- geometria -------------------------------------
# Largura limpa a cada passada (diâmetro útil do robô) e distância entre faixas.
ROBOT_DIAMETER = 0.30
LANE_SPACING = ROBOT_DIAMETER
#: O eixo das rodas fica 7 cm à frente da origem de base_link (medido na cena).
AXLE_X = 0.07

# ----------------------------- velocidades -----------------------------------
LANE_SPEED = 0.25            # avanço nas faixas [m/s]
CONTOUR_SPEED = 0.12         # avanço contornando obstáculos [m/s]
TURN_RATE = 1.0              # giro no lugar [rad/s]
MAX_ROT = 1.5                # limite de velocidade angular [rad/s]
BACK_SPEED = 0.10            # ré ao desencalhar [m/s]

# ----------------------------- controle --------------------------------------
K_HEADING = 2.5
K_WALL = 5.0
K_WALL_D = 1.5
ARC_RADIUS = 0.20
K_WALL_ANGLE = 2.0
WALL_MAX_ANGLE = math.radians(15.0)
WALL_STABLE_ERROR = 0.02
WALL_STABLE_RATE = 0.02
WALL_HEADING_GAIN = 0.02

# ----------------------------- limiares --------------------------------------
# Folgas para o corpo deste robô. Girando no lugar, a traseira (20,5 cm atrás
# do eixo, 10 cm para cada lado) descreve um círculo de ~0,23 m em torno do
# eixo das rodas. O original usava FRONT_STOP 0,20 / SIDE_TARGET 0,22 /
# SIDE_MIN 0,17, pensados para um círculo de 0,19 m: com este corpo a traseira
# raspava a parede ao girar. Como as paredes da cena não têm colisão, raspar
# era atravessar. Aqui o eixo fica a pelo menos ~0,25 m de qualquer parede vista.
FRONT_STOP = 0.27            # a partir dos sensores da frente [m] (eixo a 0,30 m)
SIDE_TARGET = 0.28           # a partir do centro do eixo [m]
SIDE_LOST = 0.46
SIDE_MIN = 0.25
TURN_TOLERANCE = math.radians(1.5)
TURN_TIMEOUT = 6.0
RETURN_MIN_OFFSET = 0.08
RETURN_MIN_ALONG = 0.10
EXHAUSTED_BACK = LANE_SPACING
MAX_FLIPS = 2
CONTOUR_MAX_PATH = 4.0
STUCK_TIME = 2.5
STUCK_DISTANCE = 0.03
BACK_TIME = 0.8
STUCK_WINDOW = 20.0
STUCK_LIMIT = 3
ESCAPE_MIN = math.radians(90.0)
ESCAPE_MAX = math.radians(180.0)

# ----------------------------- sensores virtuais -----------------------------
# (posição em base_link, direção, offset): mesmas posições do original, que
# tinha a frente em +y e a direita em +x. O offset faz o sensor lateral só
# começar a detectar depois da roda, para não enxergar o próprio robô.
_SPLAY = math.radians(10.0)
SENSOR_RANGE = 0.8
#: Meia largura do feixe de cada sensor virtual [m].
BEAM_HALF_WIDTH = 0.03
SENSORS = {
    'center': ((AXLE_X + 0.03, 0.0), (1.0, 0.0), 0.0),
    'frontLeft': ((AXLE_X + 0.03, 0.08), (math.cos(_SPLAY), math.sin(_SPLAY)), 0.0),
    'frontRight': ((AXLE_X + 0.03, -0.08), (math.cos(_SPLAY), -math.sin(_SPLAY)), 0.0),
    'sideLeft': ((AXLE_X, 0.0), (0.0, 1.0), 0.12),
    'sideRight': ((AXLE_X, 0.0), (0.0, -1.0), 0.12),
}
#: Sensores da frente no referencial do original (x para a direita, y para a
#: frente), usados para estimar a inclinação da parede.
_FRONT_GEOMETRY = {
    'center': ((0.0, 0.1), (0.0, 1.0)),
    'frontLeft': ((-0.08, 0.1), (-math.sin(_SPLAY), math.cos(_SPLAY))),
    'frontRight': ((0.08, 0.1), (math.sin(_SPLAY), math.cos(_SPLAY))),
}
MAX_WALL_SKEW = math.radians(60.0)


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def virtual_sensors(points):
    """Leituras dos cinco sensores a partir dos pontos do lidar.

    ``points`` são pontos (x, y) em base_link. Cada sensor é um feixe estreito
    que sai da sua posição na sua direção; a leitura é a distância, ao longo
    do feixe, do ponto mais próximo dentro dele (infinito se não houver).
    """
    leituras = {}
    for nome, ((sx, sy), (dx, dy), offset) in SENSORS.items():
        melhor = math.inf
        for px, py in points:
            rx, ry = px - sx, py - sy
            along = rx * dx + ry * dy
            if along < offset or along > SENSOR_RANGE:
                continue
            if abs(-rx * dy + ry * dx) <= BEAM_HALF_WIDTH and along < melhor:
                melhor = along
        leituras[nome] = melhor
    return leituras


def scan_points(ranges, angle_min, angle_increment, laser_x=0.0, laser_y=0.0):
    """Pontos (x, y) em base_link a partir de um LaserScan."""
    pontos = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_increment
        pontos.append((laser_x + r * math.cos(a), laser_y + r * math.sin(a)))
    return pontos


class Coverage:
    """Máquina de estados da varredura.

    Chame :meth:`step` a cada ciclo com o tempo, as leituras dos sensores e a
    pose do centro do eixo das rodas (x, y, rumo); ele devolve (linear,
    angular). :meth:`reset` volta ao passo 1.
    """

    def __init__(self, log=None, rng=None):
        self.log_fn = log
        self.rng = rng or random.Random()
        self.reset()

    # ------------------------------------------------------------------ util
    def log(self, msg):
        if self.log_fn:
            self.log_fn(msg)

    def reset(self, heading=None):
        self.state = 'SEEK'
        self.t_state = 0.0
        self.seek_heading = heading
        self.lane_heading = 0.0
        self.sweep_dir = 0.0
        self.follow_wall = False
        self.lane_count = 0
        self.flips_in_a_row = 0
        self.contour_origin = (0.0, 0.0)
        self.contour_max_offset = 0.0
        self.contour_path = 0.0
        self.turn_target = 0.0
        self.after_turn = None
        self.stuck_ref = None
        self.stuck_from = 'SEEK'
        self.stuck_times = []
        self.prev_side = {1: math.inf, -1: math.inf}
        self.t = 0.0
        self.cmd = (0.0, 0.0)

    def set_motors(self, lin, rot):
        self.cmd = (lin, max(-MAX_ROT, min(MAX_ROT, rot)))

    # ------------------------------------------------------------ sensores
    @property
    def dist_front(self):
        return min(self.readings['center'], self.readings['frontLeft'],
                   self.readings['frontRight'])

    @property
    def dist_left(self):
        return self.readings['sideLeft']

    @property
    def dist_right(self):
        return self.readings['sideRight']

    def side_distance(self, side):
        return self.dist_left if side > 0 else self.dist_right

    def sweep_side(self):
        return 1 if math.sin(self.sweep_dir - self.lane_heading) > 0 else -1

    def wall_heading_ahead(self, side):
        """Rumo paralelo à parede à frente, virando para o lado ``side``."""
        points = []
        for name, ((px, py), (dx, dy)) in _FRONT_GEOMETRY.items():
            d = self.readings.get(name, math.inf)
            if d != math.inf:
                points.append((px + d * dx, py + d * dy))
        skew = 0.0
        if len(points) >= 2:
            points.sort()
            (x1, y1), (x2, y2) = points[0], points[-1]
            if x2 - x1 > 0.05:
                skew = math.atan2(y2 - y1, x2 - x1)
                if abs(skew) > MAX_WALL_SKEW:
                    skew = 0.0
        return wrap(self.theta + side * math.pi / 2.0 + skew)

    # --------------------------------------------------------- comportamentos
    def hold_heading(self, heading, speed):
        rot = K_HEADING * wrap(heading - self.theta)
        if self.dist_left < SIDE_MIN:
            rot -= K_WALL * (SIDE_MIN - self.dist_left)
        if self.dist_right < SIDE_MIN:
            rot += K_WALL * (SIDE_MIN - self.dist_right)
        self.set_motors(speed, rot)

    def side_rate(self, side, dt):
        dist = self.side_distance(side)
        rate = 0.0
        if dist <= SIDE_LOST and self.prev_side[side] != math.inf and dt > 0:
            rate = (dist - self.prev_side[side]) / dt
        self.prev_side[side] = dist if dist <= SIDE_LOST else math.inf
        return rate

    def follow_wall_lane(self, side, speed, dt):
        dist = self.side_distance(side)
        rate = self.side_rate(side, dt)
        if dist > SIDE_LOST:
            return False
        deviation = K_WALL_ANGLE * (dist - SIDE_TARGET)
        deviation = max(-WALL_MAX_ANGLE, min(WALL_MAX_ANGLE, deviation))
        self.set_motors(speed, K_HEADING * wrap(
            self.lane_heading + side * deviation - self.theta))
        if abs(dist - SIDE_TARGET) < WALL_STABLE_ERROR and abs(rate) < WALL_STABLE_RATE:
            self.lane_heading = wrap(self.lane_heading + WALL_HEADING_GAIN
                                     * wrap(self.theta - self.lane_heading))
        return True

    def follow_side(self, side, speed, dt):
        dist = self.side_distance(side)
        if dist > SIDE_LOST:
            self.prev_side[side] = math.inf
            return False
        derivative = 0.0
        if self.prev_side[side] != math.inf and dt > 0:
            derivative = (dist - self.prev_side[side]) / dt
        self.prev_side[side] = dist
        self.set_motors(speed, side * (K_WALL * (dist - SIDE_TARGET)
                                       + K_WALL_D * derivative))
        return True

    # ------------------------------------------------------------ transições
    def enter(self, state):
        self.state = state
        self.t_state = 0.0
        self.stuck_ref = (self.t, self.x, self.y)

    def start_turn(self, target, then, reason):
        self.turn_target = wrap(target)
        self.after_turn = then
        self.log('%s: girando %+.0f°' % (
            reason, math.degrees(wrap(self.turn_target - self.theta))))
        self.enter('TURN')

    def start_lane(self, heading, sweep, wall):
        self.lane_heading = wrap(heading)
        self.sweep_dir = wrap(sweep)
        self.follow_wall = wall
        self.lane_count += 1
        self.flips_in_a_row = 0
        self.prev_side[1] = self.prev_side[-1] = math.inf
        self.log('faixa %d: rumo %+.0f°, varredura para a %s%s' % (
            self.lane_count, math.degrees(self.lane_heading),
            'esquerda' if self.sweep_side() > 0 else 'direita',
            ', seguindo a parede' if wall else ''))
        self.enter('LANE')

    def start_contour(self):
        self.contour_origin = (self.x, self.y)
        self.contour_max_offset = 0.0
        self.contour_path = 0.0
        self.log('faixa %d: obstáculo a %.2f m, contornando'
                 % (self.lane_count, self.dist_front))
        self.enter('CONTOUR')

    def restart(self, reason):
        side = 1 if self.dist_left >= self.dist_right else -1
        self.seek_heading = wrap(self.theta + side * self.rng.uniform(ESCAPE_MIN, ESCAPE_MAX))
        self.stuck_times = []
        self.log('%s, recomeçando a estratégia' % reason)
        self.start_turn(self.seek_heading, lambda: self.enter('SEEK'), 'saída')

    def lane_coordinates(self):
        dx = self.x - self.contour_origin[0]
        dy = self.y - self.contour_origin[1]
        along = dx * math.cos(self.lane_heading) + dy * math.sin(self.lane_heading)
        offset = dx * math.cos(self.sweep_dir) + dy * math.sin(self.sweep_dir)
        return along, offset

    def check_stuck(self):
        t0, x0, y0 = self.stuck_ref
        if self.t - t0 < STUCK_TIME:
            return False
        moved = math.hypot(self.x - x0, self.y - y0)
        self.stuck_ref = (self.t, self.x, self.y)
        if moved < STUCK_DISTANCE:
            self.log('encalhado (%.3f m em %.1f s), dando ré' % (moved, STUCK_TIME))
            self.stuck_from = self.state
            self.stuck_times = [s for s in self.stuck_times
                                if self.t - s <= STUCK_WINDOW] + [self.t]
            self.enter('BACK')
            return True
        return False

    # --------------------------------------------------------------- ciclo
    def step(self, t, dt, readings, x, y, theta):
        """Um ciclo de controle. Devolve (linear [m/s], angular [rad/s])."""
        self.t, self.readings = t, readings
        self.x, self.y, self.theta = x, y, theta
        if self.stuck_ref is None:
            self.stuck_ref = (t, x, y)
        if self.seek_heading is None:
            self.seek_heading = theta
        self._step(dt)
        self.t_state += dt
        return self.cmd

    def _step(self, dt):
        if self.state == 'SEEK':
            if self.dist_front <= FRONT_STOP:
                side = 1 if self.dist_left >= self.dist_right else -1
                heading = self.wall_heading_ahead(side)
                sweep = heading + side * math.pi / 2.0
                self.start_turn(heading, lambda: self.start_lane(heading, sweep, True),
                                'primeiro obstáculo')
            elif not self.check_stuck():
                self.hold_heading(self.seek_heading, LANE_SPEED)

        elif self.state == 'TURN':
            error = wrap(self.turn_target - self.theta)
            if abs(error) <= TURN_TOLERANCE or self.t_state >= TURN_TIMEOUT:
                self.set_motors(0.0, 0.0)
                self.after_turn()
            else:
                rot = max(-TURN_RATE, min(TURN_RATE, 3.0 * error))
                if abs(rot) < 0.25:
                    rot = math.copysign(0.25, error)
                self.set_motors(0.0, rot)

        elif self.state == 'LANE':
            if self.dist_front <= FRONT_STOP:
                self.start_contour()
                return
            if self.check_stuck():
                return
            wall_side = -self.sweep_side()
            if not (self.follow_wall and self.follow_wall_lane(wall_side, LANE_SPEED, dt)):
                self.hold_heading(self.lane_heading, LANE_SPEED)

        elif self.state == 'CONTOUR':
            sweep_side = self.sweep_side()
            obstacle_side = -sweep_side
            along, offset = self.lane_coordinates()
            self.contour_max_offset = max(self.contour_max_offset, offset)
            self.contour_path += abs(CONTOUR_SPEED) * dt

            if offset >= LANE_SPACING:
                heading = self.lane_heading + math.pi
                sweep = self.sweep_dir
                self.start_turn(heading, lambda: self.start_lane(heading, sweep, False),
                                'afastamento de %.2f m' % offset)
                return

            if (self.contour_max_offset >= RETURN_MIN_OFFSET and offset <= 0.0
                    and along >= RETURN_MIN_ALONG):
                heading = self.lane_heading
                sweep = self.sweep_dir
                self.start_turn(heading, lambda: self.start_lane(heading, sweep, False),
                                'obstáculo contornado')
                return

            front_blocked = self.dist_front <= FRONT_STOP

            if along <= -EXHAUSTED_BACK:
                flips = self.flips_in_a_row + 1
                if flips >= MAX_FLIPS:
                    self.restart('preso num corredor')
                    return
                self.log('fim da varredura nesse sentido, invertendo')
                self.start_lane(self.lane_heading + math.pi, self.sweep_dir + math.pi, True)
                self.flips_in_a_row = flips
                return

            if self.contour_path >= CONTOUR_MAX_PATH:
                self.restart('contorno longo demais')
                return

            if front_blocked:
                self.set_motors(0.0, sweep_side * TURN_RATE)
                return
            if self.check_stuck():
                return
            if not self.follow_side(obstacle_side, CONTOUR_SPEED, dt):
                self.set_motors(CONTOUR_SPEED, obstacle_side * CONTOUR_SPEED / ARC_RADIUS)

        elif self.state == 'BACK':
            if self.t_state < BACK_TIME:
                self.set_motors(-BACK_SPEED, 0.0)
            elif len(self.stuck_times) >= STUCK_LIMIT:
                self.restart('encalhes repetidos')
            elif self.stuck_from == 'SEEK':
                self.enter('SEEK')
            else:
                self.start_contour()

        else:
            self.set_motors(0.0, 0.0)
