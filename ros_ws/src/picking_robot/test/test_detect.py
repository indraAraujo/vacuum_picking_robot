"""Detecção da banana: máscara, projeção e pose, sem ROS e sem simulador.

    cd ros_ws/src/picking_robot && PYTHONPATH=. python3 -m pytest test
"""

import math

import numpy as np
import pytest

from picking_robot import detect


def banana_points(x, y, yaw, n=400, comprimento=0.21, curva=0.05):
    """Pontos do topo de uma banana no chão: um arco com o meio em (x, y).

    O arco abre para o lado +y da banana, como o modelo da cena: o meio do
    corpo fica no ponto pedido e as pontas curvam para o outro lado.
    """
    t = np.linspace(-1.0, 1.0, n)
    bx = t * comprimento / 2.0
    by = curva * t * t                      # 0 no meio, +curva nas pontas
    largura = 0.02 * np.cos(t * math.pi / 2)
    rng = np.random.default_rng(0)
    bx = np.concatenate([bx, bx])
    by = np.concatenate([by - largura, by + largura])
    by += rng.normal(0, 0.002, len(by))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + bx * c - by * s, y + bx * s + by * c], axis=1)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -1.2, 2.5, math.pi / 2])
def test_pose_acha_o_meio_do_corpo_e_a_direcao(yaw):
    pose = detect.banana_pose(banana_points(1.5, -0.4, yaw))
    assert pose is not None
    x, y, ang = pose
    assert math.hypot(x - 1.5, y + 0.4) < 0.015
    d = math.atan2(math.sin(ang - yaw), math.cos(ang - yaw))
    assert abs((math.degrees(d) + 90) % 180 - 90) < 10.0


def test_pose_nao_cai_no_centro_de_massa():
    """O centro dos pontos fica no lado côncavo; o meio do corpo, não."""
    pts = banana_points(0.0, 0.0, 0.0)
    x, y, _ = detect.banana_pose(pts)
    assert abs(y) < 0.015 < abs(pts[:, 1].mean())


def test_poucos_pontos():
    assert detect.banana_pose(np.zeros((5, 2))) is None


def test_mascara_pega_amarelo_e_ignora_o_laranja_do_braco():
    img = np.zeros((40, 60, 3), np.uint8)
    img[5:15, 5:25] = (245, 215, 60)      # banana
    img[25:35, 30:55] = (235, 130, 40)    # braço do youBot
    mask = detect.yellow_mask(img)
    assert mask[6:14, 6:24].mean() > 200
    assert mask[25:35, 30:55].max() == 0


def test_projecao_e_borrao():
    # Uma mancha amarela no meio da imagem, a 1 m, sem tocar a borda.
    rgb = np.zeros((48, 64, 3), np.uint8)
    rgb[28:36, 36:52] = (245, 215, 60)      # abaixo e à direita do centro
    depth = np.full((48, 64), 1.0, np.float32)
    achados = detect.blobs(detect.yellow_mask(rgb), min_area=20)
    assert len(achados) == 1
    us, vs = achados[0]
    pts = detect.project(us, vs, depth, 60.0, 60.0, 31.5, 23.5)
    assert len(pts) == len(us)
    assert np.allclose(pts[:, 2], 1.0)
    # O centro da mancha está à direita e abaixo do centro óptico.
    assert pts[:, 0].mean() > 0 and pts[:, 1].mean() > 0


def test_borrao_cortado_pela_borda_e_descartado():
    rgb = np.zeros((48, 64, 3), np.uint8)
    rgb[0:10, 20:30] = (245, 215, 60)
    assert detect.blobs(detect.yellow_mask(rgb), min_area=20) == []
