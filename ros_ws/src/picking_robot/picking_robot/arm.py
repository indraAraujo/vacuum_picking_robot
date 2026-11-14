"""O braço e a garra do youBot montados no robô: juntas, limites e poses.

Sem ROS e sem simulador, para dar para testar. Os nomes são os da cena
(``simulation/scene_house_arm.ttt``), herdados do modelo KUKA YouBot do
CoppeliaSim. Os limites são os das juntas da cena, em radianos.
"""

import math

#: Juntas rotativas do braço, da base para a ponta.
ARM_JOINTS = [f'youBotArmJoint{i}' for i in range(5)]
#: Juntas prismáticas dos dois dedos. O segundo segue o primeiro (script da
#: garra na cena): só se comanda "aberta" ou "fechada".
GRIPPER_JOINTS = ['youBotGripperJoint1', 'youBotGripperJoint2']

#: (mínimo, máximo) de cada junta do braço [rad].
LIMITS = [
    (-2.9496, 2.9496),
    (-1.5708, 1.3090),
    (-2.2864, 2.2864),
    (-1.7802, 1.7802),
    (-1.5708, 1.5708),
]

# ------------------------------------------------------------- geometria
# Medida na cena, no referencial do robô (y para a frente, z para cima, origem
# no centro do corpo). Com a junta 0 em zero, as juntas 1, 2 e 3 giram em
# torno de +x e o braço se move no plano y-z. Ângulo positivo leva a garra
# para trás (-y); negativo, para a frente.
#: Base do braço no topo do robô: recuada 5 cm do centro, para o Kinect caber
#: na frente (ver build_scene.py) [m].
MOUNT_Y = -0.05
#: (y, z) da junta 1 [m]: 3,3 cm à frente do eixo da base do braço.
SHOULDER = (0.033 + MOUNT_Y, 0.201)
LINKS = (0.155, 0.135, 0.209)  #: junta 1 -> 2, 2 -> 3, 3 -> ponta dos dedos [m]
#: Altura do chão no referencial do robô [m].
FLOOR_Z = -0.08


def fingertip(q):
    """(y, z) da ponta dos dedos no referencial do robô, com a junta 0 em zero."""
    y, z = SHOULDER
    phi = 0.0
    for theta, length in zip(q[1:4], LINKS):
        phi += theta
        y -= length * math.sin(phi)
        z += length * math.cos(phi)
    return y, z


def reach(y, z, pitch=-math.pi, wrist=0.0):
    """Juntas para a ponta dos dedos ir a (y, z), com a garra no ângulo ``pitch``.

    ``pitch`` é o ângulo da garra a partir da vertical (o mesmo sentido das
    juntas): -pi aponta para baixo, -pi/2 para a frente. Devolve None se o
    ponto estiver fora do alcance ou dos limites. Entre as duas soluções, fica
    a de cotovelo para cima.
    """
    l1, l2, l3 = LINKS
    # Punho: recua da ponta ao longo da garra.
    wy = y + l3 * math.sin(pitch) - SHOULDER[0]
    wz = z - l3 * math.cos(pitch) - SHOULDER[1]
    d2 = wy * wy + wz * wz
    c2 = (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if abs(c2) > 1.0:
        return None
    melhores = []
    for t2 in (math.acos(c2), -math.acos(c2)):
        # Direção do primeiro elo a partir da vertical, no sentido das juntas.
        a = math.atan2(-wy, wz)
        b = math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
        t1 = a - b
        t3 = pitch - t1 - t2
        t3 = math.atan2(math.sin(t3), math.cos(t3))
        q = [0.0, t1, t2, t3, wrist]
        if all(lo - 1e-9 <= v <= hi + 1e-9 for v, (lo, hi) in zip(q, LIMITS)):
            cotovelo = SHOULDER[1] + l1 * math.cos(t1)
            melhores.append((cotovelo, q))
    return max(melhores)[1] if melhores else None


def reach_any(y, z, wrist=0.0, pitches=(-math.pi, -2.83, -2.51, -2.2, -1.88, -1.57)):
    """Como reach, testando inclinações da garra da vertical para a horizontal."""
    for pitch in pitches:
        q = reach(y, z, pitch, wrist)
        if q is not None:
            return q
    return None


#: Poses prontas [rad]. ``repouso`` é a do modelo original: dobrado em cima do
#: robô, como na foto do youBot. ``chao`` deixa a ponta dos dedos 2 cm acima
#: do chão, 25 cm à frente do centro do robô, com a garra apontada para baixo,
#: para pegar algo; ``carregar`` ergue o que foi pego 15 cm acima do chão.
POSES = {
    'repouso': [0.0, 0.5390, 0.9150, 1.2690, 0.0],
    'alto': [0.0, 0.0, 0.0, 0.0, 0.0],
    'chao': reach(0.25, FLOOR_Z + 0.02),
    'carregar': reach(0.20, FLOOR_Z + 0.15),
}


def clamp(targets):
    """Prende cada alvo aos limites da junta; aceita 5 valores."""
    if len(targets) != len(LIMITS):
        raise ValueError(f'o braço tem {len(LIMITS)} juntas, vieram {len(targets)} valores')
    return [min(max(float(v), lo), hi) for v, (lo, hi) in zip(targets, LIMITS)]
