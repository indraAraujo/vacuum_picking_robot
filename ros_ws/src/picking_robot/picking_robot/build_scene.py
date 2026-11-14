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
   propriedade ``customData.gripperClose`` do robô (1 fecha, 0 abre). Como a
   garra não tem física, é ele que "pega": ao fechar, se houver uma banana
   (``/pickBanana``, criadas pelo nó world) entre os dedos, ela fica presa à
   mão e os dedos param na largura dela; ao abrir, ela é solta e cai. O que
   está preso sai em ``customData.holding`` (handle, ou -1);
5. cria o script ``/myRobot/rosInterface``, que a ponte ROS 2 chama para ler
   e comandar tudo numa chamada só (ver bridge.py);
6. põe no chão da sala o cesto onde o robô deixa as bananas
   (``/bananaBasket``): o *largeBasket* do CoppeliaSim, reduzido, azul e
   fixo no lugar;
7. faz o robô não colidir com as bananas (bit ``BANANA_BIT`` da máscara
   global), para ele não chutá-las ao chegar perto. Elas continuam colidindo
   com o chão, com o cesto e umas com as outras.

Rode com um CoppeliaSim aberto (pode ser sem janela). A cena aberta nele é
substituída:

    ros2 run picking_robot build_scene --port 23000
"""

import argparse
import os

import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    from picking_robot import arm
except ImportError:            # rodando o arquivo direto, fora do pacote
    import arm

#: Perfil de movimento das juntas do braço: velocidade [rad/s], aceleração e
#: solavanco. O youBot vem com 2*pi rad/s, rápido demais para acompanhar.
ARM_PROFILE = [1.0, 2.0, 20.0]
#: Base preta do braço (no youBot ela faz parte da malha da plataforma, que
#: é descartada): cilindro só visual sobre o robô [m].
BASE_DIAMETER = 0.15
BASE_HEIGHT = 0.05
#: Posição da base do braço no robô (x para a direita, y para a frente) [m]:
#: recuada, para o Kinect caber na frente.
MOUNT_XY = (0.0, arm.MOUNT_Y)
#: Kinect: pé encostado na frente do topo do robô, virado para a frente e
#: inclinado para o chão. Com 25 graus, ele vê o chão de ~0,2 m à frente do
#: robô até quase o horizonte (o sensor alcança 3,5 m).
KINECT_TILT_DEG = 25.0
#: Cesto: centro na cena (é (-0,53; -2,00) no frame map, a área mais livre da
#: sala, com ~1 m de folga para todo lado), diâmetro e altura [m].
BASKET_XY = (3.02, -1.225)
BASKET_DIAMETER = 0.40
BASKET_HEIGHT = 0.12
#: Bit da máscara global de colisão que as bananas usam e o robô não.
BANANA_BIT = 0x0200

GRIPPER_SCRIPT = '''sim = require'sim'

-- Garra do youBot comandada de fora: a propriedade customData.gripperClose do
-- /myRobot vale 1 para fechar e 0 para abrir (a ponte ROS 2 a muda). O dedo 2
-- anda em velocidade constante, e o dedo 1 o segue, como no modelo original.
--
-- A garra não tem física, então este script também "pega": fechando, quando a
-- abertura chega à largura de uma banana (/pickBanana) que esteja entre os
-- dedos, a banana é presa à mão e os dedos param. Abrindo, ela é solta e cai.

-- Abertura entre as faces dos dedos = -posição do dedo 2 [m].
GRASP_GAP = 0.040       -- largura da banana no meio
GRASP_RADIUS = 0.05     -- distância máxima do centro dos dedos ao meio da banana

function sysCall_init()
    j1 = sim.getObject('../youBotGripperJoint1')
    j2 = sim.getObject('../youBotGripperJoint2')
    palm = sim.getObject('..')
    f1 = sim.getObjectChild(j1, 0)
    f2 = sim.getObjectChild(j2, 0)
    robot = sim.getObject('/myRobot')
    held = -1
    tried = false
end

-- Ponto entre os dedos, no mundo.
function graspPoint()
    local a, b = sim.getObjectPosition(f1, -1), sim.getObjectPosition(f2, -1)
    return {(a[1] + b[1]) / 2, (a[2] + b[2]) / 2, (a[3] + b[3]) / 2}
end

-- Banana mais perto do ponto entre os dedos, se estiver ao alcance. O meio de
-- cada banana (customData.graspOffset, no referencial dela) não é o centro da
-- caixa: ela é curva.
function bananaBetweenFingers()
    local p = graspPoint()
    local best, bestD = -1, GRASP_RADIUS
    local i = 0
    while true do
        local h = sim.getObject('/pickBanana', {index = i, noError = true})
        if h < 0 then break end
        if sim.getObjectParent(h) == -1 then
            local off = sim.getFloatArrayProperty(h, 'customData.graspOffset', {noError = true}) or {0, 0, 0}
            local m = sim.getObjectMatrix(h, -1)
            local q = sim.multiplyVector(m, off)
            local d = math.sqrt((q[1] - p[1]) ^ 2 + (q[2] - p[2]) ^ 2 + (q[3] - p[3]) ^ 2)
            if d < bestD then best, bestD = h, d end
        end
        i = i + 1
    end
    return best
end

function sysCall_actuation()
    local close = sim.getIntProperty(robot, 'customData.gripperClose', {noError = true})
    if close == 1 then
        if held >= 0 then
            sim.setJointTargetVelocity(j2, 0)          -- segurando: dedos parados
        else
            sim.setJointTargetVelocity(j2, 0.04)
            if not tried and -sim.getJointPosition(j2) <= GRASP_GAP then
                tried = true
                local h = bananaBetweenFingers()
                if h >= 0 then
                    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
                    sim.setObjectParent(h, palm, true)
                    sim.resetDynamicObject(h)
                    held = h
                end
            end
        end
    else
        tried = false
        if held >= 0 then
            sim.setObjectParent(held, -1, true)
            sim.setObjectInt32Param(held, sim.shapeintparam_static, 0)
            sim.resetDynamicObject(held)
            held = -1
        end
        sim.setJointTargetVelocity(j2, -0.04)
    end
    sim.setJointTargetPosition(j1, sim.getJointPosition(j2) * -0.5)
    sim.setIntProperty(robot, 'customData.holding', held)
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

-- Pose do robô, tempo de simulação, posição de cada junta (braço e dedos) e
-- o que a garra segura (handle, ou -1).
function snapshot()
    local q = {}
    for i, h in ipairs(arm) do q[i] = sim.getJointPosition(h) end
    for i, h in ipairs(fingers) do q[#arm + i] = sim.getJointPosition(h) end
    local held = sim.getIntProperty(robot, 'customData.holding', {noError = true}) or -1
    return sim.getObjectPosition(robot, -1), sim.getObjectOrientation(robot, -1),
        sim.getSimulationTime(), q, held
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

-- Bananas pegáveis (criadas pelo nó world): handle, matriz no mundo e pai
-- (-1 no chão; a mão, se estiver na garra).
function bananas()
    local out = {}
    local i = 0
    while true do
        local h = sim.getObject('/pickBanana', {index = i, noError = true})
        if h < 0 then break end
        out[#out + 1] = {h, sim.getObjectMatrix(h, -1), sim.getObjectParent(h)}
        i = i + 1
    end
    return out
end
'''


def _add_basket(sim, model, log=print):
    """Cesto do largeBasket do CoppeliaSim: reduzido, azul, fixo no chão."""
    basket = sim.loadModel(model)
    sx = BASKET_DIAMETER / sim.getShapeBB(basket)[0][0]
    sz = BASKET_HEIGHT / sim.getShapeBB(basket)[0][2]
    for h in sim.getObjectsInTree(basket, sim.sceneobject_shape):
        sim.scaleObject(h, sx, sx, sz, 0)
        if h != basket:
            sim.setObjectPosition(h, basket, [0.0, 0.0, 0.0])
            sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, [0.1, 0.3, 0.85])
    sim.setObjectInt32Param(basket, sim.shapeintparam_static, 1)
    sim.setObjectAlias(basket, 'bananaBasket')
    sim.setObjectOrientation(basket, -1, [0.0, 0.0, 0.0])
    largura, _, altura = sim.getShapeBB(basket)[0]
    sim.setObjectPosition(basket, -1, [BASKET_XY[0], BASKET_XY[1], altura / 2])
    log(f'cesto em {BASKET_XY} (cena), {largura:.2f} m x {altura:.2f} m')


def _add_kinect(sim, model, robot, robot_top, log=print):
    """Kinect da biblioteca do CoppeliaSim na frente do topo do robô.

    Fica em /myRobot/kinect, com os sensores /myRobot/kinect/rgb e
    /myRobot/kinect/depth (os mesmos nomes do robô do potential_field). Sem
    peso e sem colisão, como o braço.
    """
    kinect = sim.loadModel(model)
    sim.setObjectAlias(kinect, 'kinect')
    sim.setObjectParent(kinect, robot, True)
    pe = sim.getShapeBB(kinect)[0]                      # o pé (x, y, z)
    for h in sim.getObjectsInTree(kinect, sim.sceneobject_shape):
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    sim.setObjectOrientation(kinect, robot, [0.0, 0.0, 0.0])
    rgb = sim.getObject('/myRobot/kinect/rgb')
    junta = sim.getObject('/myRobot/kinect/joint')
    sim.setJointPosition(junta, 0.0)
    # Gira o Kinect em torno do eixo vertical até o sensor olhar para +y (a
    # frente do robô): o sensor de visão olha ao longo do próprio eixo z.
    m = sim.getObjectMatrix(rgb, robot)
    olha = math.atan2(m[6], m[2])
    sim.setObjectOrientation(kinect, robot, [0.0, 0.0, math.pi / 2 - olha])
    frente = sim.getShapeBB(robot)[0][1] / 2.0
    sim.setObjectPosition(kinect, robot, [0.0, frente - pe[1] / 2, robot_top + pe[2] / 2])
    # Inclina pela junta do próprio Kinect, no sentido que abaixa o olhar.
    alvo = math.radians(KINECT_TILT_DEG)
    for sinal in (1.0, -1.0):
        sim.setJointPosition(junta, sinal * alvo)
        m = sim.getObjectMatrix(rgb, robot)
        if m[10] < 0:                                   # eixo z do sensor desce
            break
    m = sim.getObjectMatrix(rgb, robot)
    log(f'Kinect na frente do robô, olhando {math.degrees(math.asin(-m[10])):.0f} graus para baixo')


def build(sim, source, youbot_model, output, basket_model, kinect_model, log=print):
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
    sim.setIntProperty(robot, 'customData.holding', -1)

    # O robô não colide com as bananas.
    for h in sim.getObjectsInTree(robot, sim.sceneobject_shape):
        if sim.getObjectInt32Param(h, sim.shapeintparam_respondable):
            mask = sim.getObjectInt32Param(h, sim.shapeintparam_respondable_mask)
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable_mask, mask & ~BANANA_BIT)

    _add_basket(sim, basket_model, log)
    _add_kinect(sim, kinect_model, robot, robot_top, log)

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
    p.add_argument('--basket', default=os.path.join(
        coppelia, 'models', 'household', 'largeBasket.ttm'))
    p.add_argument('--kinect', default=os.path.join(
        coppelia, 'models', 'components', 'sensors', 'kinect.ttm'))
    p.add_argument('--output', default=os.path.join(repo, 'simulation', 'scene_house_arm.ttt'))
    a = p.parse_args(argv)
    for caminho in (a.source, a.youbot, a.basket, a.kinect):
        if not os.path.exists(caminho):
            p.error(f'não achei {caminho}')
    sim = RemoteAPIClient(port=a.port).getObject('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
    build(sim, a.source, a.youbot, a.output, a.basket, a.kinect)


if __name__ == '__main__':
    main()
