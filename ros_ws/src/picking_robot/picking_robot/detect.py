"""Achar bananas numa imagem RGB-D: segmentação, projeção e pose.

Sem ROS e sem simulador, para dar para testar. O nó vision.py junta estas
funções com as imagens do Kinect e a TF da câmera.

A banana é um arco: o ponto onde a garra a pega é o meio do corpo dela, que
não é o centro da caixa que a envolve nem o centro de massa dos pixels (os dois
caem para o lado côncavo, onde não há banana). Por isso ``banana_pose`` mede o
**eixo médio**: fatia o borrão ao longo do comprimento e toma o centro de cada
fatia. O meio desse eixo é onde a garra deve descer, e a direção dele ali é
como a banana está deitada.
"""

import math

import cv2
import numpy as np

#: Faixa do amarelo da banana em HSV do OpenCV (H 0-179). O braço do robô é
#: laranja (H ~12) e fica de fora.
HSV_LOW = (20, 120, 80)
HSV_HIGH = (35, 255, 255)


def yellow_mask(rgb):
    """Máscara dos pixels amarelos de uma imagem RGB (uint8, H x W x 3)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def blobs(mask, min_area=60, border=2):
    """Borrões da máscara: lista de (us, vs) com as colunas e linhas de cada um.

    Borrões que encostam na borda da imagem ficam de fora: estão cortados, e
    de um pedaço de banana não dá para dizer onde está o meio dela.
    """
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    h, w = mask.shape
    out = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < min_area:
            continue
        if x <= border or y <= border or x + bw >= w - border or y + bh >= h - border:
            continue
        vs, us = np.where(lab == i)
        out.append((us, vs))
    return out


def project(us, vs, depth, fx, fy, cx, cy, max_range=3.4):
    """Pixels + profundidade -> pontos 3D no referencial óptico da câmera.

    Convenção do ROS: x para a direita, y para baixo, z para a frente, com a
    linha 0 da imagem em cima.
    """
    z = depth[vs, us]
    ok = np.isfinite(z) & (z > 0.05) & (z < max_range)
    us, vs, z = us[ok], vs[ok], z[ok]
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)


def banana_pose(xy, bins=9, min_points=20):
    """(x, y, rumo) do meio da banana, a partir dos pontos dela no plano do chão.

    ``xy`` são os pontos (N x 2) num plano horizontal (o frame map). Devolve
    None se houver pontos de menos. O rumo vale módulo 180 graus: a banana é
    simétrica, e a garra só precisa cruzá-la.
    """
    if len(xy) < min_points:
        return None
    centro = xy.mean(axis=0)
    d = xy - centro
    _, vecs = np.linalg.eigh(d.T @ d)
    comprimento, largura = vecs[:, -1], vecs[:, 0]
    u, v = d @ comprimento, d @ largura

    # Centro de cada fatia ao longo do comprimento: o meio entre as bordas, e
    # não a média, que puxaria para o lado que a câmera enxerga melhor.
    bordas = np.linspace(u.min(), u.max(), bins + 1)
    eixo = []
    for k in range(bins):
        dentro = (u >= bordas[k]) & (u <= bordas[k + 1])
        if dentro.sum() < 3:
            continue
        eixo.append((u[dentro].mean(), (v[dentro].min() + v[dentro].max()) / 2.0))
    if len(eixo) < 3:
        return None
    eixo = np.array(eixo)

    # O meio do comprimento, e a direção do eixo médio ali (reta pelas fatias
    # mais próximas do meio: numa banana curva, as pontas apontam para outro
    # lado).
    meio = (u.min() + u.max()) / 2.0
    perto = np.argsort(np.abs(eixo[:, 0] - meio))[:max(3, bins // 2)]
    trecho = eixo[np.sort(perto)]
    coef = np.polyfit(trecho[:, 0], trecho[:, 1], 1)
    ponto = centro + meio * comprimento + np.polyval(coef, meio) * largura
    direcao = comprimento + coef[0] * largura
    return float(ponto[0]), float(ponto[1]), math.atan2(direcao[1], direcao[0])


def detect(rgb, depth, fx, fy, cx, cy, to_map, min_area=60, max_range=3.4,
           floor_max=0.10, exclude=None):
    """Bananas vistas na imagem: lista de (x, y, z, rumo) no frame map.

    ``to_map`` é a matriz 3x4 que leva um ponto do referencial óptico da
    câmera para o map. Só entram borrões cujos pontos estejam no chão
    (até ``floor_max`` de altura).

    ``exclude`` é um círculo (x, y, raio) a ignorar, o cesto: de perto, a
    pilha de bananas dentro dele é um borrão amarelo grande, e o meio desse
    borrão não é banana nenhuma. O teste é feito nos pontos do borrão, não no
    meio estimado, que de um monte de bananas juntas cai em qualquer lugar.
    """
    achadas = []
    for us, vs in blobs(yellow_mask(rgb), min_area):
        pts = project(us, vs, depth, fx, fy, cx, cy, max_range)
        if len(pts) < 20:
            continue
        mundo = pts @ to_map[:, :3].T + to_map[:, 3]
        z = float(np.median(mundo[:, 2]))
        if z > floor_max:
            continue
        if exclude is not None:
            dentro = np.hypot(mundo[:, 0] - exclude[0], mundo[:, 1] - exclude[1]) < exclude[2]
            if dentro.mean() > 0.25:
                continue
        pose = banana_pose(mundo[:, :2])
        if pose is not None:
            achadas.append((pose[0], pose[1], z, pose[2]))
    return achadas
