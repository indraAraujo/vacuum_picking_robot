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
│   └── human_interaction/          # Comandos de voz (Vosk) e cômodos da casa
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

- Para os comandos de voz: PortAudio, Vosk e sounddevice, também no Python do sistema (o `ros2 run` não usa ambientes virtuais):

  ```bash
  sudo apt install libportaudio2
  /usr/bin/python3 -m pip install --user --break-system-packages vosk sounddevice
  ```

  Na primeira execução, o Vosk baixa o modelo pequeno de português (31 MB) para `~/.cache/vosk/`.

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
colcon build --symlink-install --packages-select vacuum_cleaner human_interaction
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

O launch sobe os seis nós do robô, o Nav2 e o **mapa pronto** da casa (`ros_ws/src/vacuum_cleaner/maps/casa.yaml`), publicado pelo `map_server`. O robô já começa conhecendo o mapa. O sistema está pronto quando a mensagem `Managed nodes are active` aparecer três vezes: mapa, nós do robô e Nav2.

O robô precisa começar na posição da cena salva (`scene_house.ttt`), porque o mapa usa essa posição como origem.

Para mapear com o SLAM Toolbox em vez de usar o mapa salvo, por exemplo depois de mudar a cena:

```bash
ros2 launch vacuum_cleaner vacuum_cleaner.launch.py slam:=true
```

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

### Refazer o mapa

O mapa salvo foi gerado automaticamente pelo `house_mapper`: com o SLAM ligado, ele leva o robô a cada cômodo de `rooms.py` pelo Nav2, dá uma volta de 360° em cada um (o lidar só vê 180° à frente) e salva o mapa e o estado do SLAM. Para refazer:

1. Suba o projeto com o SLAM: `ros2 launch vacuum_cleaner vacuum_cleaner.launch.py slam:=true`
2. Em outro terminal, na raiz do repositório:

   ```bash
   ros2 run human_interaction house_mapper --ros-args \
     -p output:=$PWD/ros_ws/src/vacuum_cleaner/maps/casa -p use_sim_time:=true
   ```

3. Recompile o `vacuum_cleaner` para instalar o mapa novo e suba o projeto de novo, sem `slam:=true`.

Na simulação sem janela, o percurso pelos cinco cômodos levou cerca de 8 minutos. Cômodo que não dá para alcançar é anotado e pulado depois de `goal_timeout` (padrão: 600 s), então cada cômodo inalcançável soma até 10 minutos.

Também dá para completar o mapa à mão: suba com `slam:=true`, dirija pelo teclado até as áreas que faltam e salve com:

```bash
ros2 run nav2_map_server map_saver_cli -f ros_ws/src/vacuum_cleaner/maps/casa --ros-args -p use_sim_time:=true
```

Salve antes de fechar o launch, porque o mapa em construção fica só na memória. O comando sobrescreve `casa.pgm` e `casa.yaml`.

#### Cobertura do mapa atual

O mapa cobre a **sala de jantar, a cozinha, a sala, o lavabo e o corredor**. Entrada, banheiro e quarto ficaram de fora porque, na cena, as portas deles estão **fechadas**: o laser vê as portas como parede, e o Nav2 não tem por onde entrar. Para incluí-los, abra essas portas na cena, refaça o mapa e devolva os três cômodos em `rooms.py` (as coordenadas estão comentadas lá).

As paredes da cena não têm colisão física (`respondable` desligado). Com o mapa, o Nav2 não manda o robô por elas, mas comandos diretos, como os do teclado ou da limpeza, ainda podem atravessá-las.

### Comandos de voz

Com o projeto rodando, abra outro terminal, carregue o ambiente e rode:

```bash
ros2 launch human_interaction voice.launch.py
```

O microfone já fica ouvindo. Fale, por exemplo, **"vá para a cozinha"**: o robô transcreve a frase, interpreta o comando e manda o destino para o Nav2.

| Comando | O que o robô faz |
|---|---|
| "vá para a cozinha" | Vai até o cômodo |
| "limpar a cozinha" | Vai até o cômodo pelo Nav2 e começa a varredura em faixas lá |
| "voltar para a base" | Volta ao ponto onde começou |
| "limpar tudo", "limpar aqui" | Começa a varredura onde está |
| "mapear a casa" | Percorre todos os cômodos, um a um |
| "pausar" / "continuar" | Para sem perder a fila e retoma de onde parou |
| "parar" | Cancela tudo e para onde está |
| "como está" | Diz onde está, para onde vai e quanto falta |

Os cômodos são: **sala de jantar, cozinha, sala, lavabo e corredor**, os que estão no mapa (veja "Cobertura do mapa atual").

O robô transcreve tudo o que ouve, mas só age quando a frase é um comando. As outras falas aparecem no log como `ouvido, mas não é tarefa minha`.

Opções úteis:

```bash
ros2 launch human_interaction voice.launch.py console:=1        # Digitar as frases, sem microfone
ros2 launch human_interaction voice.launch.py wake_word:=robo   # Só obedece depois de "robô"
ros2 launch human_interaction voice.launch.py device:=8         # Outra entrada de áudio
ros2 launch human_interaction voice.launch.py lidar:=true       # Manter o laser ligado
ros2 run human_interaction voice_listener --ros-args -p list_devices:=true   # Listar entradas
```

Para acompanhar a execução:

```bash
ros2 topic echo /myRobot/voice/transcript   # O que foi ouvido
ros2 topic echo /myRobot/voice/command      # O comando interpretado
ros2 topic echo /myRobot/house/room         # O cômodo atual
ros2 topic echo /myRobot/house/task         # A tarefa atual
ros2 topic echo /myRobot/voice/level        # Nível do microfone (acima de -40 dBFS ao falar)
```

#### O lidar no modo de voz

Para poupar processamento, o modo de voz **desliga o laser** na cena enquanto estiver rodando e o religa ao sair (Ctrl+C). O laser é o que mais pesa: sem ele, a cena sem janela passou de cerca de 0,4 para cerca de 7 vezes o tempo real.

Junto com o laser:

- o `myrobot_server` publica um `/scan` vazio, sem nenhum eco. Sem ele, o `collision_monitor` para o robô por falta de dados;
- se o projeto estiver com `slam:=true`, o SLAM Toolbox **pausa** as medições novas e o mapa fica congelado. O scan vazio também mantém viva a TF `map → odom` do SLAM, que ele carimba com o horário do último scan.

Com o laser desligado, o Nav2 navega só pelo mapa já construído e **não vê obstáculos novos**. Por isso, mapeie a casa antes de usar a voz. Para manter o laser ligado:

```bash
ros2 launch human_interaction voice.launch.py lidar:=true
```

O laser também pode ser ligado e desligado à mão:

```bash
ros2 service call /myRobot/lidar/enable std_srvs/srv/SetBool "{data: false}"
```

Ao subir, o `myrobot_server` sempre religa o laser, caso uma sessão anterior tenha terminado sem religá-lo.

Os destinos e os limites dos cômodos ficam em `ros_ws/src/human_interaction/human_interaction/rooms.py`, no referencial da cena. Se o robô começar em outro ponto da cena, atualize `SCENE_ORIGIN` nesse arquivo.

### Limpeza

No modo `CLEANING`, o `cleaning_server` varre o ambiente em **faixas paralelas** (boustrophedon), com a estratégia do controlador `2 - python_controller.py` do projeto *4 - roomba*:

1. anda reto até o primeiro obstáculo;
2. gira até ficar paralelo a ele e segue a parede: é a primeira faixa;
3. contorna os obstáculos sempre para o lado ainda não limpo;
4. quando se afasta um diâmetro do robô (0,30 m) da faixa anterior, gira e começa a faixa seguinte no sentido oposto, a curva em "U".

Também trata obstáculos estreitos (retoma a mesma faixa), o fim da sala (inverte o lado da varredura) e armadilhas (gira para o lado mais livre e recomeça).

Os cinco sensores de proximidade do original são **virtuais**. Por padrão, o `cleaning_server` os calcula **no mapa, sem o laser**: a pose vem da simulação (exata), e cada sensor vira três raios percorridos no `/map` até a primeira célula ocupada (`vacuum_cleaner/map_sensors.py`). Célula desconhecida ou fora do mapa conta como parede. Assim o laser fica desligado também durante a limpeza.

Para usar o laser em vez do mapa (por exemplo, para enxergar objetos que não estão no mapa):

```bash
ros2 launch vacuum_cleaner vacuum_cleaner.launch.py cleaning_sensors:=lidar
ros2 launch human_interaction voice.launch.py lidar:=true
```

A lógica da varredura fica em `vacuum_cleaner/coverage.py`, que não depende de ROS. Os testes em `test/test_coverage.py` a rodam numa sala sintética com as duas fontes.

Iniciar e parar pelo serviço da máquina de estados:

```bash
ros2 service call /myRobot/start_stop vacuum_cleaner_interfaces/srv/StartStop '{command: "start"}'
ros2 service call /myRobot/start_stop vacuum_cleaner_interfaces/srv/StartStop '{command: "stop"}'
```

Ou por voz: "limpar a sala" leva o robô até a sala pelo Nav2 e começa a varredura lá; "limpar tudo" começa onde ele está; "parar" encerra.

Pontos de atenção:

- **Bateria:** a bateria simulada gasta 1 % por segundo de simulação limpando e carrega 2 % por segundo na base. Com 25 %, a máquina de estados manda o robô para a base, e com 100 % ela volta a limpar. Ajuste com os parâmetros `battery_drain` e `battery_charge` do `myrobot_server`.
- **Limites da varredura:** ela é reativa e não fica presa ao cômodo. Pelas portas abertas, o robô pode seguir para outros cômodos.
- **Folga das paredes:** as distâncias de parada foram aumentadas em relação ao original, porque a traseira deste robô gira num círculo maior (~0,23 m em torno do eixo). Como as paredes da cena não têm colisão, raspar nelas era atravessar. O custo é uma faixa de ~13 cm junto às paredes que não é varrida.
- **Sensores pelo mapa:** só enxergam o que está no mapa. Objetos novos no chão e portas que o laser não registrou no mapeamento passam despercebidos. Sem mapa, o robô fica parado; com `cleaning_sensors:=lidar`, ele também para se o scan estiver vazio ou atrasado.
- **Velocidade da simulação:** sem o laser, a cena roda várias vezes mais rápido que o tempo real. Por isso o `myrobot_server` aplica cada comando de velocidade assim que ele chega. Antes, ele esperava um timer de 50 ms de relógio, o que virava ~0,25 s de simulação com a velocidade antiga, e o robô passava do ponto de parada.

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
| `cleaning_server` | Varredura em faixas no modo `CLEANING` | Assina `/map` (ou `/scan`, com `cleaning_sensors:=lidar`) e `/myRobot/odom`; publica `/myRobot/cmd_vel` |
| `autodocking_server` | Aproximação da base no modo `AUTODOCKING` | Lê o sinal da base e publica em `/myRobot/cmd_vel` |
| `charging_server` | Reservado para a supervisão da carga | Assina a bateria e o modo |
| `charged_server` | Reservado para tarefas com o robô na base | Assina o modo |
| `cmd_vel_mux` | Prioridade do teclado sobre os demais comandos | Assina `/myRobot/cmd_vel_teleop` e `/myRobot/cmd_vel`; publica `/myRobot/cmd_vel_mux`, que o `myrobot_server` assina |
| `voice_listener` | Microfone → texto, com o Vosk offline em português | Publica `/myRobot/voice/transcript` |
| `command_parser` | Texto → comando (intenção e lugar) | Assina a transcrição; publica `/myRobot/voice/command` |
| `voice_commander` | Comando → destino no Nav2 | Assina o comando; usa a ação `/navigate_to_pose`; publica `/myRobot/house/room` e `/myRobot/house/task` |
| `map_server` | Mapa pronto (padrão) | Publica `/map` a partir de `maps/casa.yaml`; a TF `map → odom` é fixa |
| `slam_toolbox` | Mapeamento (`slam:=true`) | Assina `/scan`; publica `/map` e a TF `map → odom` |
| `house_mapper` | Mapeamento automático | Visita os cômodos pelo Nav2, gira em cada um e salva o mapa |
| Nav2 | Planejamento e controle de trajetória | Oferece `/navigate_to_pose`; a saída do `collision_monitor` vai para `/myRobot/cmd_vel` |

### Ciclos de vida

Três gerenciadores cuidam da inicialização:

- `/lifecycle_manager`: nós do robô (lista em `config/nav2_params.yaml`).
- `/lifecycle_manager_navigation`: nós do Nav2.
- `/lifecycle_manager_map`: o `map_server`, quando o projeto usa o mapa pronto.

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

- A limpeza e o docking não usam o Nav2: os dois publicam velocidades direto em `/myRobot/cmd_vel`. Durante a limpeza, a voz recusa comandos de deslocamento até você dizer "parar". O docking segue uma base simulada, sem relação com uma base real na cena.
- A bateria e o sinal da base são simulados dentro do `myrobot_server`, sem relação com a posição real da base.
- O lidar cobre 180° à frente do robô.
- O mapa não cobre entrada, banheiro e quarto (portas fechadas na cena), e as paredes da cena não têm colisão.

## Referências

- [Base de arquitetura para navegação, por Luciano](https://github.com/lucianovk/IA368II/tree/main)
