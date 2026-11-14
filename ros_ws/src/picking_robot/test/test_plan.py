"""Geometria de pegar e entregar, sem ROS e sem simulador.

    cd ros_ws/src/picking_robot && PYTHONPATH=. python3 -m pytest test
"""

import math

import pytest

from picking_robot.plan import approach_points, in_robot, nearest, wrist_for


@pytest.mark.parametrize('banana', [0.0, 0.4, 1.4, -2.0, math.pi, 3.0])
@pytest.mark.parametrize('robo', [0.0, 1.0, -2.5])
def test_punho_dentro_dos_limites_e_atravessado(banana, robo):
    a = wrist_for(banana, robo)
    assert -math.pi / 2 - 1e-9 <= a <= math.pi / 2 + 1e-9
    # Direção de fechamento (lateral do robô girada de -a) perpendicular à banana.
    fecha = robo + math.pi / 2 - a
    assert abs(math.cos(fecha - banana)) < 1e-9


def test_in_robot():
    frente, lado = in_robot((1.0, 1.0, math.pi / 2), (1.0, 1.5))
    assert frente == pytest.approx(0.5) and lado == pytest.approx(0.0, abs=1e-12)
    frente, lado = in_robot((0.0, 0.0, 0.0), (0.0, 0.3))
    assert lado == pytest.approx(0.3)


def _sala(x, y):
    """Distância à parede numa sala de 4 x 3 m com uma parede em x = 2 m (y < 2)."""
    d = min(x, 4 - x, y, 3 - y)
    if y < 2.0:
        d = min(d, abs(x - 2.0))
    return d


def test_pontos_de_chegada_livres_e_virados_para_o_alvo():
    alvo = (1.0, 1.0)
    pontos = approach_points(alvo, 0.65, 0.27, _sala, robot_xy=(3.0, 2.5))
    assert pontos
    for x, y, yaw in pontos:
        assert _sala(x, y) >= 0.30
        assert math.hypot(x - alvo[0], y - alvo[1]) == pytest.approx(0.65)
        assert math.atan2(alvo[1] - y, alvo[0] - x) == pytest.approx(yaw)
    # O mais perto do robô vem primeiro.
    d = [math.hypot(x - 3.0, y - 2.5) for x, y, _ in pontos]
    assert d == sorted(d)


def test_evita_o_cesto():
    pontos = approach_points((1.0, 1.0), 0.65, 0.27, _sala, robot_xy=(0, 0),
                             avoid=[(1.0, 1.65, 0.2)])
    for x, y, _ in pontos:
        assert math.hypot(x - 1.0, y - 1.65) >= 0.2 + 0.30


def test_nearest():
    assert nearest([(0, 0), (1, 1)], (0.9, 0.9)) == (1, 1)
    assert nearest([(0, 0)], (1, 1), max_dist=0.5) is None
