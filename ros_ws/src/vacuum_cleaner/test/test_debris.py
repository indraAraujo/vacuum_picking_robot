"""Sorteio das posições da sujeira, sem ROS e sem simulador.

    cd ros_ws/src/vacuum_cleaner && PYTHONPATH=. python3 -m pytest test/test_debris.py
"""

import math
import os
import random

from vacuum_cleaner.debris import FloorMap, read_pgm, sample_spots

MAPA = os.path.join(os.path.dirname(__file__), '..', 'maps', 'casa.yaml')


def _sala(fechado=False):
    """Sala de 3 x 2 m (células de 5 cm) com um quartinho isolado à direita."""
    w, h = 100, 40
    livre = [[False] * w for _ in range(h)]
    for j in range(1, h - 1):
        for i in range(1, 59):
            livre[j][i] = True
        for i in range(61, w - 1):       # quartinho atrás da parede x = 3,0 m
            livre[j][i] = True
    return FloorMap(w, h, 0.05, 0.0, 0.0, livre)


def test_folga_das_paredes():
    sala = _sala()
    assert abs(sala.clearance(1.0, 1.0) - 0.95) < 0.05     # 1 m da parede de baixo
    assert sala.clearance(0.07, 1.0) < 0.06


def test_so_onde_o_robo_alcanca_e_longe_das_paredes():
    pontos = sample_spots(_sala(), 30, start=(0.5, 1.0), rng=random.Random(3))
    assert pontos
    for x, y in pontos:
        assert x < 3.0, f'banana no quartinho fechado: ({x:.2f}, {y:.2f})'
        assert 0.30 <= x <= 2.70 and 0.30 <= y <= 1.70, (x, y)
        assert math.hypot(x - 0.5, y - 1.0) >= 0.6
    assert all(math.hypot(a[0] - b[0], a[1] - b[1]) >= 0.4
               for k, a in enumerate(pontos) for b in pontos[k + 1:])


def test_mapa_da_casa():
    casa = FloorMap.from_yaml(MAPA)
    w, h, _ = read_pgm(casa_pgm := MAPA.replace('.yaml', '.pgm'))
    assert (casa.width, casa.height) == (w, h), casa_pgm
    pontos = sample_spots(casa, 15, rng=random.Random(1))
    assert len(pontos) == 15
    assert min(casa.clearance(x, y) for x, y in pontos) >= 0.30
