"""A varredura presa a um cômodo: não sai pela porta aberta.

Duas salas lado a lado (0..4 m e 4..7 m), separadas por uma parede em x = 4 m
com uma porta aberta de 0,8 m. A área de limpeza é só a primeira sala.

    cd ros_ws/src/vacuum_cleaner && PYTHONPATH=. python3 -m pytest test/test_area.py
"""

import math
import random

from vacuum_cleaner.coverage import AXLE_X, Coverage
from vacuum_cleaner.map_sensors import Area, MapSensors, area_readings

RES = 0.05
W, H = 140, 60                       # 7 x 3 m
PORTA = (1.1, 1.9)                   # vão da porta em y, na parede x = 4 m
SALA_1 = Area([(0.0, 4.0, 0.0, 3.0)])
#: Corpo do robô em base_link, como em test_coverage.py.
BODY = [(bx, by) for bx in (-0.135, 0.0, 0.14) for by in (-0.10, 0.0, 0.10)]


def _parede(x, y):
    if not (0.0 < x < 7.0 and 0.0 < y < 3.0):
        return True
    return 3.95 <= x <= 4.05 and not (PORTA[0] <= y <= PORTA[1])


def _mapa():
    data = []
    for j in range(H):
        for i in range(W):
            data.append(100 if _parede((i + 0.5) * RES, (j + 0.5) * RES) else 0)
    return MapSensors(data, W, H, RES, 0.0, 0.0)


def _run(x, y, th, area, seconds=500.0, dt=0.05, seed=2, sensores=None):
    mapa = _mapa()
    sensores = sensores or (lambda x, y, th: mapa.readings(x, y, th, area))
    cov = Coverage(rng=random.Random(seed))
    max_x, celulas, trajeto = -math.inf, set(), []
    for k in range(int(seconds / dt)):
        ax, ay = x + AXLE_X * math.cos(th), y + AXLE_X * math.sin(th)
        if area is not None:
            area.follow(x, y)
        lin, rot = cov.step(k * dt, dt, sensores(x, y, th), ax, ay, th)
        th += rot * dt
        x += lin * math.cos(th) * dt
        y += lin * math.sin(th) * dt
        c, s = math.cos(th), math.sin(th)
        max_x = max(max_x, max(x + bx * c - by * s for bx, by in BODY))
        trajeto.append((x, y))
        for i in range(-3, 4):
            for j in range(-3, 4):
                if i * i + j * j <= 9:
                    celulas.add((int(ax / RES) + i, int(ay / RES) + j))
    livres = {(i, j) for i in range(80) for j in range(60)
              if not _parede((i + 0.5) * RES, (j + 0.5) * RES)
              and SALA_1.contains((i + 0.5) * RES, (j + 0.5) * RES)}
    return max_x, len(celulas & livres) / len(livres), trajeto


def test_sem_area_o_robo_passa_pela_porta():
    max_x, _, _ = _run(1.0, 1.5, 0.0, None)
    assert max_x > 4.2, 'o cenário não testa nada: sem área ele já não sairia'


def test_com_area_o_robo_fica_no_comodo():
    max_x, cobertura, _ = _run(1.0, 1.5, 0.0, Area(SALA_1.rects))
    assert max_x < 4.0, f'o corpo do robô passou da porta: x = {max_x:.2f} m'
    assert cobertura >= 0.70, f'cobertura do cômodo de {cobertura:.0%}'


def test_fora_da_area_ele_volta_e_fica():
    # Começa 0,3 m além da porta, fora do cômodo.
    area = Area(SALA_1.rects)
    _, _, trajeto = _run(4.3, 1.5, math.pi, area, seconds=400.0)
    dentro = [k for k, (x, _) in enumerate(trajeto) if x <= 4.0]
    assert dentro, 'nunca voltou para o cômodo'
    depois = trajeto[dentro[0] + 200:]          # 10 s depois de entrar
    assert max(x for x, _ in depois) < 4.15, 'voltou a sair do cômodo'


def test_area_vista_como_parede_pelo_lidar():
    # Na fonte lidar, a borda da área entra como leitura extra.
    leituras = area_readings(Area(SALA_1.rects), 3.5, 1.5, 0.0)
    assert abs(leituras['center'] - (4.0 - 3.6)) < 0.03
    assert Area.from_flat([]) is None


def test_parede_virtual_nao_acompanha_quem_se_afasta():
    area = Area([(0.0, 4.0, 0.0, 3.0)])
    assert area.follow(4.3, 1.5) > 0 and abs(area.margin - 0.4) < 1e-9
    area.follow(4.8, 1.5)                 # afastou-se: a margem não cresce
    assert abs(area.margin - 0.4) < 1e-9
    area.follow(4.1, 1.5)                 # voltando: ela encolhe atrás dele
    assert abs(area.margin - 0.2) < 1e-9
    area.follow(3.9, 1.5)                 # dentro: some
    assert area.margin == 0.0
