# Robô Aspirador com Garra e Interação Humano-Robô

Arquitetura embarcada para um robô aspirador com garra capaz de pegar lixo, mapear ambientes e interagir com seres humanos.

O projeto usa ROS 2 Jazzy e simula o robô no CoppeliaSim. Hoje, esta branch tem a navegação autônoma (SLAM Toolbox e Nav2) e a máquina de estados de limpeza.

## Estrutura do repositório

```text
.
├── ros_ws/src/
│   ├── vacuum_cleaner/             # Nós do robô, launch e configurações
│   │   ├── config/                 # Parâmetros do Nav2 e do SLAM Toolbox
│   │   ├── launch/                 # vacuum_cleaner.launch.py
│   │   └── vacuum_cleaner/         # Código dos nós
│   ├── vacuum_cleaner_interfaces/  # Serviço StartStop
│   └── human_interaction/          # Pacote de interação (esqueleto)
└── simulation/
    └── scene_house.ttt             # Cena do robô no CoppeliaSim
```

## Pré-requisitos

- ROS 2 Jazzy
- Nav2 e SLAM Toolbox:

  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
  ```

- CoppeliaSim
- Cliente Python da Remote API, instalado no Python do sistema:

  ```bash
  pip install --user coppeliasim-zmqremoteapi-client
  ```

## Como compilar

Rode os comandos na raiz do repositório.

**1. Compile as interfaces em um caminho simples.** O `rosidl` não consegue gerar interfaces em caminhos como o deste repositório, que tem acentos e espaços. Se o seu caminho também tiver, compile uma cópia do pacote fora do repositório:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/vacuum_interfaces_ws/src
cp -r ros_ws/src/vacuum_cleaner_interfaces ~/vacuum_interfaces_ws/src/
(cd ~/vacuum_interfaces_ws && colcon build)
```

Refaça esse passo sempre que alterar o arquivo `.srv`.

**2. Compile o pacote do robô:**

```bash
source ~/vacuum_interfaces_ws/install/setup.bash
colcon build --symlink-install --packages-select vacuum_cleaner
```

Depois de editar arquivos em `config/` ou `launch/`, compile de novo para que a alteração valha.

## Como executar

**1. Abra a cena.** No CoppeliaSim, abra `simulation/scene_house.ttt`. Não é preciso dar play: o `myrobot_server` inicia a simulação se ela estiver parada.

**2. Carregue o ambiente.** Faça isso em cada terminal novo, na raiz do repositório:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vacuum_interfaces_ws/install/setup.bash
source install/setup.bash
```

**3. Suba o projeto:**

```bash
ros2 launch vacuum_cleaner vacuum_cleaner.launch.py
```

O launch sobe os seis nós do robô, o SLAM Toolbox e o Nav2. O sistema está pronto quando a mensagem `Managed nodes are active` aparecer duas vezes: uma para os nós do robô e outra para o Nav2.

**4. Abra o RViz (opcional):**

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

### Mandar o robô para um ponto

No RViz, use o botão **Nav2 Goal**. Pelo terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}"
```

As coordenadas estão no frame `map`, com origem no ponto em que o robô começou e `x` apontando para a frente dele.

### Controlar pelo teclado

Com o projeto rodando, abra outro terminal, carregue o ambiente e rode:

```bash
ros2 launch vacuum_cleaner teleop.launch.py
```

Com a janela `teleop` em foco, use `i`/`,` para andar para a frente e para trás, `j`/`l` para girar, `k` para parar e `q`/`z` para mudar a velocidade. O teclado tem prioridade sobre o Nav2 e sobre os nós de limpeza.

### Continuar o mapeamento

O SLAM Toolbox constrói o mapa enquanto o robô anda, e o mapa em construção fica só na memória. Para completar o mapa:

1. Mantenha o launch principal aberto e dirija o robô pelo teclado até as áreas que faltam.
2. Gire o robô nos cômodos e passe perto dos cantos, porque o lidar só cobre 180° à frente.
3. Acompanhe o resultado no RViz.
4. Salve o mapa em outro terminal, com o ambiente carregado, na raiz do repositório:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ros_ws/src/vacuum_cleaner/maps/casa --ros-args -p use_sim_time:=true
   ```

O comando sobrescreve `casa.pgm` e `casa.yaml`. Salve antes de fechar o launch.

Esses arquivos são apenas uma imagem do mapa: o SLAM não consegue continuar a partir deles. Ao reiniciar o launch, o mapeamento recomeça do zero. Para retomar em outra sessão, salve também o estado do SLAM antes de fechar:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'casa_slam'}"
```

Esse comando grava `casa_slam.data` e `casa_slam.posegraph` na pasta em que o launch foi iniciado. Na sessão seguinte, com a cena reiniciada e o robô na posição inicial, suba o launch a partir da mesma pasta e carregue esse estado:

```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: 'casa_slam', match_type: 1}"
```

O `match_type: 1` retoma o mapa a partir do ponto em que o mapeamento começou.

### Iniciar e parar a limpeza

```bash
ros2 service call /myRobot/start_stop vacuum_cleaner_interfaces/srv/StartStop '{command: "start"}'
ros2 service call /myRobot/start_stop vacuum_cleaner_interfaces/srv/StartStop '{command: "stop"}'
```

### Acompanhar a execução

```bash
ros2 topic echo /myRobot/behaviour/mode     # Modo atual
ros2 topic echo /myRobot/battery_state      # Bateria simulada
ros2 topic echo /myRobot/odom               # Odometria
```

### Desempenho da simulação

Em máquinas com placa NVIDIA híbrida, abra o CoppeliaSim pela placa dedicada, com a janela normal:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ./coppeliaSim.sh
```

O laser é o outro gargalo. O parâmetro `scanning_density` do `myrobot_server` define os raios por grau, e o padrão é `1.0` (1° de resolução).

| Configuração (cena `scene_house.ttt`) | Tempo real |
|---|---|
| Placa integrada, densidade 2.5 | 5% |
| NVIDIA, densidade 2.5 | 21% |
| NVIDIA, densidade 1.0 | 45% |

Sem interface (`coppeliaSim.sh -h`), a velocidade chegou a 51% com densidade 1.0 e a 92% com 0.5.

O SLAM e o Nav2 seguem o tempo da simulação (`/clock`), então continuam corretos abaixo do tempo real.

## Arquitetura

### Nós

| Nó | Função | Principais entradas e saídas |
| --- | --- | --- |
| `myrobot_server` | Ponte com o CoppeliaSim e telemetria simulada | Assina `/myRobot/cmd_vel_mux` e comanda os motores; publica `/clock`, `/scan`, `/myRobot/odom`, a TF `odom → base_link`, a bateria e o sinal da base |
| `behaviour_server` | Máquina de estados de operação | Assina a bateria e o sinal da base; oferece `/myRobot/start_stop`; publica `/myRobot/behaviour/mode` |
| `cleaning_server` | Movimento de limpeza no modo `CLEANING` | Publica uma velocidade fixa em `/myRobot/cmd_vel` |
| `autodocking_server` | Aproximação da base no modo `AUTODOCKING` | Lê o sinal da base e publica em `/myRobot/cmd_vel` |
| `charging_server` | Reservado para a supervisão da carga | Assina a bateria e o modo |
| `charged_server` | Reservado para tarefas com o robô na base | Assina o modo |
| `cmd_vel_mux` | Prioridade do teclado sobre os demais comandos | Assina `/myRobot/cmd_vel_teleop` e `/myRobot/cmd_vel`; publica `/myRobot/cmd_vel_mux`, que o `myrobot_server` assina |
| `slam_toolbox` | Mapeamento e localização | Assina `/scan`; publica `/map` e a TF `map → odom` |
| Nav2 | Planejamento e controle de trajetória | Oferece `/navigate_to_pose`; a saída do `collision_monitor` vai para `/myRobot/cmd_vel` |

### Ciclos de vida

Dois gerenciadores cuidam da inicialização, e ambos usam `config/nav2_params.yaml`:

- `/lifecycle_manager`: nós do robô.
- `/lifecycle_manager_navigation`: nós do Nav2.

O ciclo de vida só define se um nó está ativo. O que o robô faz em cada momento é decidido pelo modo publicado pelo `behaviour_server`.

### Máquina de estados

O `behaviour_server` começa em `CHARGED` e usa a bateria, o sinal da base e o serviço `start_stop` para mudar de modo:

```text
CHARGED     --(start)-----------------------> CLEANING
CLEANING    --(bateria <= 25% ou stop)------> AUTODOCKING
AUTODOCKING --(sinal da base forte)---------> CHARGING
CHARGING    --(bateria cheia, com stop)-----> CHARGED
CHARGING    --(bateria cheia, sem stop)-----> CLEANING
```

Cada transição aparece no log com o motivo, por exemplo `Behaviour mode CLEANING -> AUTODOCKING (battery low (0.24))`. A cada 5 s, o nó também registra a bateria e o modo atual.

## Comandos de operação

Os comandos abaixo usam o gerenciador dos nós do robô. Para o Nav2, troque `/lifecycle_manager` por `/lifecycle_manager_navigation`.

```bash
# Verificar se os nós estão ativos
ros2 service call /lifecycle_manager/is_active std_srvs/srv/Trigger "{}"

# Pausar, retomar, reiniciar ou encerrar
ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"  # Pausar
ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"  # Retomar
ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"  # Reiniciar
ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"  # Encerrar
```

## Limitações atuais

- A limpeza e o docking ainda não usam o Nav2: esses nós publicam velocidades diretamente em `/myRobot/cmd_vel`. Não mande goals nos modos `CLEANING` e `AUTODOCKING`, porque os comandos se misturam.
- A bateria e o sinal da base são simulados dentro do `myrobot_server`, sem relação com a posição real da base.
- O lidar cobre 180° à frente do robô.

## Referências

- [Base de arquitetura para navegação, por Luciano](https://github.com/lucianovk/IA368II/tree/main)
