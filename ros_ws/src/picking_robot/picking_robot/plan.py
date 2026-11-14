"""Geometria de pegar e entregar, sem ROS e sem simulador.

Tudo no frame ``map`` (x, y em metros, ângulos em radianos), com o robô no
referencial do ROS: a frente é +x.
"""

import math


def wrap(a):
    """Ângulo em (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def wrist_for(banana_yaw, robot_yaw):
    """Ângulo da junta 4 para pegar a banana atravessada.

    Com a garra apontada para baixo e a junta 4 em zero, os dedos fecham na
    direção lateral do robô; girar a junta 4 de ``a`` gira essa direção de
    ``-a``. A banana precisa ficar perpendicular à direção de fechamento, ou
    seja, com o comprimento na direção da frente do robô girada de ``-a``.
    Como a banana é simétrica, vale o ângulo módulo pi, e sempre há uma
    solução em [-pi/2, pi/2], dentro dos limites da junta.
    """
    a = wrap(robot_yaw - banana_yaw)
    if a > math.pi / 2:
        a -= math.pi
    elif a < -math.pi / 2:
        a += math.pi
    return a


def in_robot(pose, point):
    """(à frente, à esquerda) de ``point`` em relação ao robô em ``pose``."""
    x, y, th = pose
    dx, dy = point[0] - x, point[1] - y
    return (dx * math.cos(th) + dy * math.sin(th),
            -dx * math.sin(th) + dy * math.cos(th))


def approach_points(target, radius, stop, clearance, robot_xy, n=16, min_clear=0.30,
                    path_clear=0.18, avoid=()):
    """Pontos de onde chegar a ``target``, do mais perto do robô ao mais longe.

    Cada ponto fica a ``radius`` do alvo, com o robô virado para ele. Dali o
    robô anda em linha reta até ficar a ``stop`` do alvo; esse trecho também
    precisa estar livre. ``clearance(x, y)`` dá a distância até a parede mais
    próxima (o mapa). ``avoid`` é uma lista de círculos (x, y, raio) a evitar,
    como o cesto. Devolve [(x, y, yaw)].
    """
    tx, ty = target
    pontos = []
    for k in range(n):
        ang = 2 * math.pi * k / n
        cx, cy = tx + radius * math.cos(ang), ty + radius * math.sin(ang)
        if clearance(cx, cy) < min_clear:
            continue
        ok = True
        passos = max(2, int((radius - stop) / 0.05))
        for i in range(passos + 1):
            d = stop + (radius - stop) * i / passos
            px, py = tx + d * math.cos(ang), ty + d * math.sin(ang)
            if clearance(px, py) < path_clear:
                ok = False
                break
            for ax, ay, ar in avoid:
                if (ax, ay) != (tx, ty) and math.hypot(px - ax, py - ay) < ar + 0.30:
                    ok = False
                    break
            if not ok:
                break
        if ok:
            pontos.append((cx, cy, wrap(ang + math.pi)))
    rx, ry = robot_xy
    pontos.sort(key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
    return pontos


def nearest(points, xy, max_dist=math.inf):
    """O ponto de ``points`` mais perto de ``xy`` (a até ``max_dist``), ou None."""
    melhor, dm = None, max_dist
    for p in points:
        d = math.hypot(p[0] - xy[0], p[1] - xy[1])
        if d < dm:
            melhor, dm = p, d
    return melhor
