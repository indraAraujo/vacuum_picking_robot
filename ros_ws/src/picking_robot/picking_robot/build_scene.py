"""Monta a cena do robô com garra a partir da cena do aspirador.

Parte de ``simulation/scene_house.ttt`` (que não é alterada) e grava
``simulation/scene_house_arm.ttt``:

1. tira o laser (/myRobot/LaserScanner_2D) do robô;
2. carrega o modelo KUKA YouBot do CoppeliaSim, solta dele só o braço com a
   garra (a partir de youBotArmJoint0) e o prende no topo do robô, no lugar do
   laser, na mesma orientação em relação à frente que tinha no youBot, sobre
   um cilindro preto que faz o papel da base do braço. A base do youBot é
   descartada;
3. deixa o braço sem peso: os elos ficam estáticos e sem colisão, e as
   juntas, em modo cinemático. O braço segue os ângulos comandados (com o
   perfil de movimento ``ARM_PROFILE``), sem massa e sem fazer força sobre o
   robô, que anda como o aspirador original. O tamanho e o formato ficam os
   do youBot. Por isso a garra não segura objetos pela física: para pegar
   algo, o objeto precisa ser preso a ela por código;
4. troca o script da garra, que no youBot só abria, por um que obedece à
   propriedade ``customData.gripperClose`` do robô (1 fecha, 0 abre);
5. cria o script ``/myRobot/rosInterface``, que a ponte ROS 2 chama para ler
   e comandar tudo numa chamada só (ver bridge.py).

Rode com um CoppeliaSim aberto (pode ser sem janela). A cena aberta nele é
substituída:

    ros2 run picking_robot build_scene --port 23000
"""

import argparse
import os

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

#: Perfil de movimento das juntas do braço: velocidade [rad/s], aceleração e
#: solavanco. O youBot vem com 2*pi rad/s, rápido demais para acompanhar.
ARM_PROFILE = [1.0, 2.0, 20.0]
#: Base preta do braço (no youBot ela faz parte da malha da plataforma, que
#: é descartada): cilindro só visual sobre o robô [m].
BASE_DIAMETER = 0.15
BASE_HEIGHT = 0.05
#: Posição da base do braço no robô (x para a direita, y para a frente) [m].
MOUNT_XY = (0.0, 0.0)

GRIPPER_SCRIPT = '''sim = require'sim'

-- Garra do youBot comandada de fora: a propriedade customData.gripperClose do
-- /myRobot vale 1 para fechar e 0 para abrir (a ponte ROS 2 a muda). O dedo 2
-- anda em velocidade constante, e o dedo 1 o segue, como no modelo original.
function sysCall_init()
    j1 = sim.getObject('../youBotGripperJoint1')
    j2 = sim.getObject('../youBotGripperJoint2')
    robot = sim.getObject('/myRobot')
end

function sysCall_actuation()
    local close = sim.getIntProperty(robot, 'customData.gripperClose', {noError = true})
    if close == 1 then
        sim.setJointTargetVelocity(j2, 0.04)
    else
        sim.setJointTargetVelocity(j2, -0.04)
    end
    sim.setJointTargetPosition(j1, sim.getJointPosition(j2) * -0.5)
end
'''

INTERFACE_SCRIPT = '''sim = require'sim'

-- Interface da ponte ROS 2 (picking_robot/bridge.py): cada função é uma ida e
-- volta só pela Remote API.
function sysCall_init()
    robot = sim.getObject('/myRobot')
    arm = {}
    for i = 0, 4 do
        arm[i + 1] = sim.getObject('/myRobot/youBotArmJoint' .. i)
    end
    fingers = {sim.getObject('/myRobot/youBotGripperJoint1'),
               sim.getObject('/myRobot/youBotGripperJoint2')}
    wheels = {sim.getObject('/myRobot/leftMotor'), sim.getObject('/myRobot/rightMotor')}
end

-- Pose do robô, tempo de simulação e posição de cada junta (braço e dedos).
function snapshot()
    local q = {}
    for i, h in ipairs(arm) do q[i] = sim.getJointPosition(h) end
    for i, h in ipairs(fingers) do q[#arm + i] = sim.getJointPosition(h) end
    return sim.getObjectPosition(robot, -1), sim.getObjectOrientation(robot, -1),
        sim.getSimulationTime(), q
end

function setArm(targets)
    for i, h in ipairs(arm) do sim.setJointTargetPosition(h, targets[i]) end
end

function setGripper(close)
    sim.setIntProperty(robot, 'customData.gripperClose', close and 1 or 0)
end

function setWheels(left, right)
    sim.setJointTargetVelocity(wheels[1], left)
    sim.setJointTargetVelocity(wheels[2], right)
end
'''


def build(sim, source, youbot_model, output, log=print):
    sim.loadScene(source)
    robot = sim.getObject('/myRobot')

    laser = sim.getObject('/myRobot/LaserScanner_2D')
    sim.removeObjects(sim.getObjectsInTree(laser))
    log('laser removido')

    # Topo do corpo do robô, no referencial dele (a caixa é centrada nele).
    robot_top = sim.getShapeBB(robot)[0][2] / 2.0
    base = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder, [BASE_DIAMETER, BASE_DIAMETER, BASE_HEIGHT], 0)
    sim.setObjectAlias(base, 'armBase')
    sim.setObjectInt32Param(base, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(base, sim.shapeintparam_respondable, 0)
    sim.setShapeColor(base, None, sim.colorcomponent_ambient_diffuse, [0.12, 0.12, 0.12])
    sim.setObjectParent(base, robot, True)
    sim.setObjectPosition(base, robot, [MOUNT_XY[0], MOUNT_XY[1], robot_top + BASE_HEIGHT / 2])
    sim.setObjectOrientation(base, robot, [0.0, 0.0, 0.0])

    youbot = sim.loadModel(youbot_model)
    joint0 = sim.getObject('/youBot/youBotArmJoint0')
    # Pose do braço num frame alinhado ao mundo, na posição do youBot. O frame
    # do próprio modelo é girado (nele, "para cima" é o eixo x), mas no mundo o
    # youBot olha para +y, como o robô no referencial dele.
    rel = list(sim.getObjectMatrix(joint0, -1))
    rel[3], rel[7] = MOUNT_XY
    rel[11] = robot_top + BASE_HEIGHT
    sim.setObjectParent(joint0, robot, True)
    sim.setObjectMatrix(joint0, sim.multiplyMatrices(sim.getObjectMatrix(robot, -1), rel), -1)
    sim.removeObjects([h for h in sim.getObjectsInTree(youbot)])
    log(f'braço preso no robô, {rel[11]:.3f} m acima do centro dele')

    arm = sim.getObjectsInTree(joint0)
    elos = juntas = 0
    for h in arm:
        tipo = sim.getObjectType(h)
        if tipo == sim.sceneobject_shape:
            # Estático: fora da física, só acompanha o pai (o robô ou o elo
            # anterior). Sem colisão: não empurra nada nem é empurrado.
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
            elos += 1
        elif tipo == sim.sceneobject_joint:
            sim.setJointMode(h, sim.jointmode_kinematic, 0)
            sim.setFloatArrayProperty(h, 'maxVelAccelJerk', ARM_PROFILE)
            juntas += 1
    log(f'braço sem peso: {elos} peças estáticas e sem colisão, {juntas} juntas cinemáticas')

    scripts = [h for h in arm if sim.getObjectType(h) == sim.sceneobject_script]
    for s in scripts:
        sim.setStringProperty(s, 'code', GRIPPER_SCRIPT)
    log(f'{len(scripts)} script(s) da garra trocados')

    iface = sim.createScript(sim.scripttype_customization, INTERFACE_SCRIPT, 0, 'lua')
    sim.setObjectParent(iface, robot, True)
    sim.setObjectAlias(iface, 'rosInterface')
    sim.setIntProperty(robot, 'customData.gripperClose', 0)

    sim.saveScene(output)
    log(f'cena salva em {output}')


def _repo():
    """Raiz do repositório: a primeira pasta com simulation/scene_house.ttt,
    subindo a partir do diretório atual e depois a partir deste arquivo."""
    for inicio in (os.getcwd(), os.path.dirname(os.path.realpath(__file__))):
        d = inicio
        while True:
            if os.path.exists(os.path.join(d, 'simulation', 'scene_house.ttt')):
                return d
            pai = os.path.dirname(d)
            if pai == d:
                break
            d = pai
    return os.getcwd()


def main(argv=None):
    repo = _repo()
    coppelia = os.environ.get('COPPELIASIM_ROOT_DIR',
                              os.path.expanduser('~/Softwares/CoppeliaSim/CoppeliaSim_Edu'))
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--port', type=int, default=23000, help='porta da ZeroMQ Remote API')
    p.add_argument('--source', default=os.path.join(repo, 'simulation', 'scene_house.ttt'))
    p.add_argument('--youbot', default=os.path.join(
        coppelia, 'models', 'robots', 'mobile', 'KUKA YouBot.ttm'))
    p.add_argument('--output', default=os.path.join(repo, 'simulation', 'scene_house_arm.ttt'))
    a = p.parse_args(argv)
    for caminho in (a.source, a.youbot):
        if not os.path.exists(caminho):
            p.error(f'não achei {caminho}')
    sim = RemoteAPIClient(port=a.port).getObject('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
    build(sim, a.source, a.youbot, a.output)


if __name__ == '__main__':
    main()
