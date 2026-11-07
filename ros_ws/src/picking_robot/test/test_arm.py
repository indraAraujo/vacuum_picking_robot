"""Juntas, limites e cinemática do braço, sem ROS e sem simulador.

    cd ros_ws/src/picking_robot && PYTHONPATH=. python3 -m pytest test
"""

import math

import pytest

from picking_robot import arm


def test_poses_prontas_dentro_dos_limites():
    for nome, q in arm.POSES.items():
        assert q is not None, nome
        assert arm.clamp(q) == pytest.approx(q), nome


def test_chao_deixa_a_garra_acima_do_chao_a_frente():
    y, z = arm.fingertip(arm.POSES['chao'])
    assert 0.20 <= y <= 0.30            # à frente do robô
    assert 0.0 < z - arm.FLOOR_Z <= 0.03


@pytest.mark.parametrize('y, z', [(0.25, -0.06), (0.20, 0.07), (0.30, -0.02), (0.22, 0.10)])
def test_reach_volta_pela_cinematica_direta(y, z):
    q = arm.reach(y, z)
    assert q is not None
    fy, fz = arm.fingertip(q)
    assert fy == pytest.approx(y, abs=1e-6) and fz == pytest.approx(z, abs=1e-6)
    assert sum(q[1:4]) == pytest.approx(-math.pi, abs=1e-6)   # garra para baixo


def test_fora_do_alcance():
    assert arm.reach(1.0, 0.0) is None


def test_clamp():
    lo, hi = arm.LIMITS[1]
    assert arm.clamp([0, 9, 0, 0, 0])[1] == hi
    assert arm.clamp([0, -9, 0, 0, 0])[1] == lo
    with pytest.raises(ValueError):
        arm.clamp([0, 0, 0])
