"""Testes da varredura em faixas, sem ROS e sem simulador.

Um simulador mínimo: sala retangular com uma mesa no meio, lidar de 180° por
raycast e cinemática diferencial perfeita.

    cd ros_ws/src/vacuum_cleaner && PYTHONPATH=. python3 -m pytest test/test_coverage.py
"""

import math
import random

from vacuum_cleaner.coverage import AXLE_X, Coverage, scan_points, virtual_sensors
from vacuum_cleaner.map_sensors import MapSensors

ROOM = (0.0, 4.0, 0.0, 3.0)          # x_min, x_max, y_min, y_max [m]
TABLE = (1.6, 2.4, 1.2, 1.8)
BOXES = [TABLE]


def _ray_box(ox, oy, dx, dy, box, inside):
    """Distância do raio até a borda da caixa (por dentro ou por fora)."""
    x0, x1, y0, y1 = box
    best = math.inf
    for (px, py, nx, ny, lo, hi, axis) in (
            (x0, 0, 1, 0, y0, y1, 'x'), (x1, 0, 1, 0, y0, y1, 'x'),
            (0, y0, 0, 1, x0, x1, 'y'), (0, y1, 0, 1, x0, x1, 'y')):
        if axis == 'x' and abs(dx) > 1e-9:
            t = (px - ox) / dx
            if t > 1e-6 and lo <= oy + t * dy <= hi:
                best = min(best, t)
        if axis == 'y' and abs(dy) > 1e-9:
            t = (py - oy) / dy
            if t > 1e-6 and lo <= ox + t * dx <= hi:
                best = min(best, t)
    return best


def _scan(x, y, th, n=181):
    ranges = []
    for i in range(n):
        a = th - math.pi / 2 + i * math.pi / (n - 1)
        dx, dy = math.cos(a), math.sin(a)
        r = _ray_box(x, y, dx, dy, ROOM, True)
        for b in BOXES:
            r = min(r, _ray_box(x, y, dx, dy, b, False))
        ranges.append(r if r <= 10.0 else math.inf)
    return ranges


def _inside(x, y, margin=0.0):
    x0, x1, y0, y1 = ROOM
    if not (x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin):
        return False
    tx0, tx1, ty0, ty1 = TABLE
    return not (tx0 - margin < x < tx1 + margin and ty0 - margin < y < ty1 + margin)


#: Corpo do robô em base_link: x de -0,135 a 0,14 m, y de -0,10 a 0,10 m.
BODY = [(bx, by) for bx in (-0.135, 0.0, 0.14) for by in (-0.10, 0.0, 0.10)]


def _lidar(x, y, th):
    return virtual_sensors(scan_points(_scan(x, y, th), -math.pi / 2, math.pi / 180))


def _mapa(res=0.05, borda=0.5):
    """OccupancyGrid da sala: paredes ocupadas, chão livre, fora desconhecido."""
    x0, x1, y0, y1 = ROOM
    ox, oy = x0 - borda, y0 - borda
    w, h = round((x1 - x0 + 2 * borda) / res), round((y1 - y0 + 2 * borda) / res)
    data = []
    for j in range(h):
        for i in range(w):
            cx, cy = ox + (i + 0.5) * res, oy + (j + 0.5) * res
            if _inside(cx, cy):
                data.append(0)
            elif x0 - res <= cx <= x1 + res and y0 - res <= cy <= y1 + res:
                data.append(100)      # parede (ou a mesa)
            else:
                data.append(-1)
    return MapSensors(data, w, h, res, ox, oy)


def _run(seconds=600.0, dt=0.05, seed=1, sensores=_lidar):
    cov = Coverage(rng=random.Random(seed))
    x, y, th = 0.8, 0.6, 0.3          # base_link
    cells = set()
    min_clear = math.inf
    toques = 0
    for k in range(int(seconds / dt)):
        ax, ay = x + AXLE_X * math.cos(th), y + AXLE_X * math.sin(th)
        lin, rot = cov.step(k * dt, dt, sensores(x, y, th), ax, ay, th)
        th += rot * dt
        x += lin * math.cos(th) * dt
        y += lin * math.sin(th) * dt
        x0, x1, y0, y1 = ROOM
        min_clear = min(min_clear, x - x0, x1 - x, y - y0, y1 - y)
        c, s = math.cos(th), math.sin(th)
        if any(not _inside(x + bx * c - by * s, y + bx * s + by * c) for bx, by in BODY):
            toques += 1
        # Célula de 5 cm a até 0,15 m do centro do eixo conta como limpa.
        cx, cy = x + AXLE_X * math.cos(th), y + AXLE_X * math.sin(th)
        for i in range(-3, 4):
            for j in range(-3, 4):
                if i * i + j * j <= 9:
                    cells.add((int(cx / 0.05) + i, int(cy / 0.05) + j))
    livres = {(i, j) for i in range(80) for j in range(60)
              if _inside((i + 0.5) * 0.05, (j + 0.5) * 0.05)}
    return len(cells & livres) / len(livres), min_clear, cov, toques


def test_varre_a_maior_parte_da_sala_sem_sair_dela():
    cobertura, folga, cov, toques = _run()
    assert folga > 0.05, f'o robô chegou a {folga:.2f} m da parede'
    assert toques == 0, f'o corpo do robô tocou parede ou mesa em {toques} passos'
    assert cov.lane_count >= 5, f'só {cov.lane_count} faixas'
    assert cobertura >= 0.70, f'cobertura de {cobertura:.0%}'


def test_varre_so_com_o_mapa_sem_lidar():
    cobertura, folga, cov, toques = _run(sensores=_mapa().readings)
    assert folga > 0.05, f'o robô chegou a {folga:.2f} m da parede'
    assert toques == 0, f'o corpo do robô tocou parede ou mesa em {toques} passos'
    assert cov.lane_count >= 5, f'só {cov.lane_count} faixas'
    assert cobertura >= 0.70, f'cobertura de {cobertura:.0%}'


def test_mapa_e_lidar_concordam():
    mapa = _mapa()
    for x, y, th in ((3.4, 1.5, 0.0), (0.5, 0.4, 2.5), (1.3, 1.5, 0.0), (2.0, 2.6, -1.2)):
        pelo_mapa, pelo_lidar = mapa.readings(x, y, th), _lidar(x, y, th)
        for nome, d in pelo_lidar.items():
            if math.isfinite(d):
                assert abs(pelo_mapa[nome] - d) < 0.06, (x, y, th, nome, pelo_mapa[nome], d)
            else:
                assert pelo_mapa[nome] > 0.7, (x, y, th, nome, pelo_mapa[nome])


def test_fim_do_mapa_conta_como_parede():
    mapa = MapSensors([0] * 100, 10, 10, 0.05, 0.0, 0.0)    # 0,5 x 0,5 m livres
    assert mapa.readings(0.1, 0.25, 0.0)['center'] < 0.35


def test_sensor_virtual_ve_parede_a_frente():
    pts = scan_points(_scan(3.4, 1.5, 0.0), -math.pi / 2, math.pi / 180)
    leituras = virtual_sensors(pts)
    # Parede em x = 4: o sensor da frente fica 0,10 m à frente de base_link.
    assert abs(leituras['center'] - (4.0 - 3.5)) < 0.02
    assert leituras['sideLeft'] == math.inf      # parede de cima a 1,5 m: fora do alcance
