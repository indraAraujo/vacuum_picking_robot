# picking_robot

O mesmo robô aspirador, na mesma casa, mas com o **braço e a garra do KUKA youBot** no lugar do lidar. Este pacote traz a cena, uma ponte ROS 2 com o CoppeliaSim e a teleoperação pelo teclado. Navegação, voz e limpeza ficam no `vacuum_cleaner` e ainda não foram trazidas para cá.

## A cena

`simulation/scene_house_arm.ttt` é gerada a partir de `simulation/scene_house.ttt`, que não muda, pelo `build_scene`:

1. tira o laser (`/myRobot/LaserScanner_2D`);
2. carrega o modelo *KUKA YouBot* do CoppeliaSim e solta dele só o braço com a garra (de `youBotArmJoint0` em diante), na mesma orientação em relação à frente que ele tinha no youBot. O braço vai no topo do robô, no lugar do laser, sobre um cilindro preto que faz o papel da base dele. O resto do youBot é descartado;
3. deixa o braço **sem peso**: as peças ficam estáticas e sem colisão, e as juntas, em modo cinemático. O braço segue os ângulos comandados, mas não faz força sobre o robô, que anda como o aspirador original;
4. troca o script da garra, que no youBot só abria, por um que abre e fecha sob comando;
5. cria o script `/myRobot/rosInterface`, por onde a ponte lê e comanda tudo numa chamada só.

Para gerar a cena de novo (por exemplo, depois de mudar `build_scene.py`), com um CoppeliaSim aberto. A cena aberta nele é substituída:

```bash
ros2 run picking_robot build_scene            # Porta 23000; --port para outra
```

## Como executar

Compile e carregue o ambiente como no README principal, incluindo `picking_robot` no `colcon build`. Depois:

1. No CoppeliaSim, abra `simulation/scene_house_arm.ttt`. A ponte dá play se a simulação estiver parada.
2. Suba a ponte:

   ```bash
   ros2 launch picking_robot picking_robot.launch.py
   ```

3. Em outro terminal, a teleoperação:

   ```bash
   ros2 run picking_robot teleop
   ```

   ```text
   Base                      Braço (junta: + / -)
     i       frente            1 / q   junta 0 (gira a base do braço)
   j k l     girar / parar     2 / w   junta 1 (ombro)
     ,       ré                3 / e   junta 2 (cotovelo)
                               4 / r   junta 3 (punho)
                               5 / t   junta 4 (gira a garra)
   Garra                     Poses prontas
     espaço  abrir / fechar    z repouso   x chão   c carregar   v alto
   ```

## Tópicos

| Tópico | Tipo | O que é |
| --- | --- | --- |
| `/myRobot/cmd_vel` | `geometry_msgs/Twist` | Velocidade da base. Sem comando por 0,6 s de simulação, as rodas param |
| `/myRobot/arm/command` | `std_msgs/Float64MultiArray` | Alvo das 5 juntas do braço [rad], preso aos limites |
| `/myRobot/arm/pose` | `std_msgs/String` | Pose pronta: `repouso`, `alto`, `chao` ou `carregar` |
| `/myRobot/gripper/command` | `std_msgs/Bool` | `true` fecha a garra, `false` abre |
| `/joint_states` | `sensor_msgs/JointState` | As 5 juntas do braço [rad] e os 2 dedos [m] |
| `/myRobot/odom` e TF `odom → base_link` | `nav_msgs/Odometry` | Pose exata da simulação, com a origem onde o robô começou |
| `/clock` | `rosgraph_msgs/Clock` | Tempo da simulação |

Exemplos sem o teleop:

```bash
ros2 topic pub --once /myRobot/arm/pose std_msgs/msg/String "{data: chao}"
ros2 topic pub --once /myRobot/gripper/command std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /myRobot/arm/command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## O braço

`picking_robot/arm.py` tem os nomes, os limites e as poses, além da cinemática no plano do braço, medida na cena:

- `fingertip(q)` diz onde fica a ponta dos dedos, no referencial do robô (y para a frente, z para cima);
- `reach(y, z)` devolve as juntas para a ponta ir a um ponto, com a garra apontada para baixo.

A pose `chao` deixa a ponta dos dedos 2 cm acima do chão, 25 cm à frente do centro do robô. Na simulação, ela ficou a 6 mm do previsto.

Os testes rodam sem ROS e sem simulador:

```bash
cd ros_ws/src/picking_robot && PYTHONPATH=. python3 -m pytest test
```

## Limitações

- **A garra não pega nada pela física:** sem peso e sem colisão, ela não empurra nem prende objetos. Para pegar algo, o objeto precisa ser preso a ela por código.
- **O braço atravessa o que estiver no caminho**, inclusive o chão e o próprio robô. As poses prontas evitam isso; os comandos junta a junta, não.
- **Sem lidar:** a cena não tem laser, então não há `/scan`.
- **O robô escorrega devagar quando parado** (~17 mm em 20 s de simulação). A cena original faz o mesmo, então não é efeito do braço.
