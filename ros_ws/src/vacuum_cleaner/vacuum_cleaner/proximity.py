"""Os cinco sensores de proximidade da limpeza, na cena, como no 4 - roomba.

A cena só tem o sensor frontal (/myRobot/proximitySensor). Como no controlador
original, este módulo cria cópias dele nas laterais e nos cantos da frente ao
subir e as remove ao sair, sem alterar a cena salva.

Ler cinco sensores pela Remote API custaria cinco idas e voltas ao simulador
por leitura. Por isso também é criado um script no robô (``snapshot``) que
devolve numa chamada só a pose, o tempo de simulação e as cinco leituras.

Referencial do /myRobot: a frente é +y e a direita é +x (o base_link do ROS
tem a frente em +x). As posições e direções são as do original.
"""

import math

#: Prefixo dos objetos criados, para achar sobras de uma execução anterior.
PREFIX = 'vacuumProx_'
SCRIPT_ALIAS = 'vacuumSensors'
SENSOR_RANGE = 0.8
_AXLE_Y = 0.07
_SPLAY = math.radians(10.0)
#: (nome, posição x, y no /myRobot, direção x, y, offset). O offset faz o
#: sensor lateral só começar a detectar depois da roda.
COPIES = [
    ('sideLeft', 0.0, _AXLE_Y, -1.0, 0.0, 0.12),
    ('sideRight', 0.0, _AXLE_Y, 1.0, 0.0, 0.12),
    ('frontLeft', -0.08, 0.1, -math.sin(_SPLAY), math.cos(_SPLAY), 0.0),
    ('frontRight', 0.08, 0.1, math.sin(_SPLAY), math.cos(_SPLAY), 0.0),
]
#: Ordem das leituras no snapshot; 'center' é o sensor frontal da cena.
NAMES = ['center'] + [c[0] for c in COPIES]
OFFSETS = {'center': 0.0, **{c[0]: c[5] for c in COPIES}}

_SNAPSHOT = '''
function snapshot(robot, sensors)
    local out = {}
    for i, h in ipairs(sensors) do
        local res, dist = sim.readProximitySensor(h)
        out[i] = (res > 0) and dist or -1
    end
    return sim.getObjectPosition(robot, -1), sim.getObjectOrientation(robot, -1),
        sim.getSimulationTime(), out
end
'''


def remove_leftovers(sim, robot_path='/myRobot'):
    """Tira cópias e script deixados por uma execução que não terminou bem."""
    restos = []
    for alias in [PREFIX + c[0] for c in COPIES] + [SCRIPT_ALIAS]:
        k = 0
        while True:
            h = sim.getObject(f'{robot_path}/{alias}', {'index': k, 'noError': True})
            if h < 0:
                break
            restos.append(h)
            k += 1
    if restos:
        sim.removeObjects(restos)
    return len(restos)


def create(sim, robot, front):
    """Cria as cópias do sensor frontal e o script; devolve (script, handles)."""
    z = sim.getObjectPosition(front, robot)[2]
    handles = [front]
    for nome, px, py, dx, dy, offset in COPIES:
        h = sim.copyPasteObjects([front], 0)[0]
        sim.setObjectParent(h, robot, True)
        sim.setObjectAlias(h, PREFIX + nome)
        # O sensor detecta ao longo do seu eixo z: z = direção, x = vertical.
        zx, zy, zz = dx, dy, 0.0
        xx, xy, xz = 0.0, 0.0, 1.0
        yx, yy, yz = zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx
        sim.setObjectMatrix(h, [xx, yx, zx, px, xy, yy, zy, py, xz, yz, zz, z], robot)
        sim.setFloatProperty(h, 'volume_offset', offset)
        sim.setFloatProperty(h, 'volume_range', SENSOR_RANGE)
        handles.append(h)
    script = sim.createScript(sim.scripttype_customization, _SNAPSHOT, 0, 'lua')
    sim.setObjectParent(script, robot, True)
    sim.setObjectAlias(script, SCRIPT_ALIAS)
    return script, handles


def readings(raw):
    """Lista do snapshot (-1 = nada detectado) -> {nome: distância ou inf}."""
    return {n: (d if d is not None and d > 0 else math.inf) for n, d in zip(NAMES, raw)}
