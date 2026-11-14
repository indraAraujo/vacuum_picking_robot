# picking_robot

O mesmo robô aspirador, na mesma casa, mas com o **braço e a garra do KUKA youBot** no lugar do lidar. Bananas ficam espalhadas pelo chão, e o robô as recolhe, uma a uma, para um cesto azul que fica no chão da sala.

Onde as bananas estão pode vir de dois lugares: da própria simulação (o padrão) ou do **Kinect** montado na frente do robô, com `vision:=true`. Nos dois casos sai o mesmo tópico, no mesmo formato, e o resto do sistema não muda. Veja [O Kinect](#o-kinect-achar-as-bananas-pela-imagem).

## Como executar

Compile e carregue o ambiente como no README principal, incluindo `picking_robot` no `colcon build`. Depois:

1. No CoppeliaSim, abra `simulation/scene_house_arm.ttt`. A ponte dá play se a simulação estiver parada.
2. Suba tudo: a ponte, as bananas, o Nav2 e o recolhimento.

   ```bash
   ros2 launch picking_robot picking_robot.launch.py
   ```

   O robô começa sozinho. Cada banana entregue aparece no log (`banana no cesto: 3/8`), e no fim sai `terminei: 8 bananas no cesto`.

| Argumento | Padrão | O que faz |
| --- | --- | --- |
| `bananas` | `8` | Quantas bananas espalhar |
| `seed` | `-1` | Semente do sorteio das posições (`-1`: um sorteio diferente a cada vez) |
| `positions` | | Bananas em posições fixas no `map`, no lugar do sorteio: `positions:="[x1, y1, x2, y2]"` |
| `vision` | `false` | `true`: as bananas vêm do Kinect (nó `vision`); `false`: da simulação (nó `world`) |
| `picker` | `true` | `false` sobe tudo, menos o recolhimento (para usar o teleop) |
| `sim_port` | `23000` | Porta da ZeroMQ Remote API do CoppeliaSim |

Para pausar e retomar o recolhimento:

```bash
ros2 service call /picking/stop std_srvs/srv/Trigger
ros2 service call /picking/start std_srvs/srv/Trigger
```

### Pelo teclado

Com `picker:=false`, num terminal à parte:

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

Fechar a garra com uma banana entre os dedos também a pega.

## Como o robô recolhe as bananas

O nó `picker` repete, da banana mais próxima à mais distante:

1. **Ir até a banana.** O Nav2 leva o robô a um ponto a 0,65 m dela, já virado para ela. O lado é escolhido para que o trecho final, em linha reta, esteja livre de paredes e longe do cesto (`plan.py`). Se o Nav2 não chegar, ele tenta outro lado.
2. **Encostar.** Um controle próprio gira o robô para a banana e anda reto até ela ficar 25 cm à frente do centro dele, com erro de menos de 1 cm. Ele usa a posição mais recente da banana, não a do início: com o Kinect, a estimativa melhora à medida que o robô chega perto.
3. **Pegar.** O punho gira para os dedos ficarem atravessados na banana (`wrist_for`). A garra desce aberta, primeiro por cima e depois até 1 cm do chão (`arm.reach`), e então fecha e sobe. O picker confirma a pega por `/myRobot/gripper/holding`. O robô volta de ré pelo mesmo corredor até 0,65 m da banana.
4. **Entregar.** O Nav2 leva o robô a 0,90 m do cesto, e o controle próprio encosta até o centro do cesto ficar 42 cm à frente. O braço estende a banana sobre o cesto, 24 cm acima do chão, e a garra abre: a banana cai dentro. O robô volta de ré até 0,90 m do cesto e segue para a próxima.

O recuo até o ponto de onde o Nav2 o trouxe é o que impede o robô de travar. Recuando pouco (30 cm), ele partia da zona de custo em volta do cesto. Se a próxima banana estivesse do outro lado do cesto, o controlador contornava o cesto rente demais e entrava na zona proibida do costmap, de onde nenhuma trajetória é válida. Por segurança, antes de cada navegação, o picker também afasta o robô do cesto se ele estiver perto demais.

O braço viaja erguido (pose `alto`), que é a pose mais compacta. Uma banana que caia fora do cesto volta a aparecer em `/bananas` e é recolhida de novo. Depois de duas tentativas sem sucesso numa mesma banana, o robô desiste dela.

**Testes na cena (8 bananas cada):** três arranjos, `seed:=11`, `seed:=23` e um que travava (uma banana atrás do cesto). Em todos, as 8 bananas foram para o cesto, em 3 a 4 minutos de relógio e uns 55 m percorridos, sem nenhuma falha do controlador. O centro do robô ficou a pelo menos 0,26 m das paredes do mapa e a 0,43 m do centro do cesto.

### Como a garra "pega"

O braço não tem peso nem colisão (veja [A cena](#a-cena)), então não dá para pegar pela física. Quem pega é o script da garra, na própria cena:

- **Ao fechar:** quando a abertura chega à largura da banana (4 cm), ele procura uma banana a até 5 cm do ponto entre os dedos. Se achar, prende a banana à mão e para os dedos ali.
- **Ao abrir:** ele solta a banana, que volta a ter física e cai.

O script não sabe qual banana o robô quer. Basta haver uma entre os dedos. Por isso o picker não precisa da identidade das bananas, só da posição delas, o que continua valendo quando a posição vier de uma câmera.

## Bananas, cesto e mapa: o nó `world`

- **As bananas.** São sorteadas no chão livre da casa, a pelo menos 0,45 m das paredes (para o robô chegar de frente), longe da base e do cesto, em lugares que o robô alcança. Cada uma é uma caixa invisível com física, com o modelo da banana por cima, só visual. Elas não colidem com o robô, que não as chuta ao chegar perto, mas colidem com o chão, com o cesto e entre si. Ao encerrar, saem da cena.
- **`/bananas`.** Publica as que estão no chão: nem na garra, nem dentro do cesto. A posição é o meio da banana, que é curva (o meio fica 3 cm fora do centro da caixa), e o ângulo é o do comprimento dela.
- **`/basket` e `/map`.** Publica a pose do cesto e o mapa da casa (`maps/casa.yaml` do `vacuum_cleaner`) com o cesto marcado como obstáculo, para o Nav2 desviar dele.
- **`/bananas/delivered`.** Conta as bananas dentro do cesto, inclusive as empilhadas acima da borda.

## O Kinect: achar as bananas pela imagem

Por padrão, quem diz onde estão as bananas é o nó `world`, que lê a posição exata da simulação. Com `vision:=true`, quem diz é o **Kinect**: o nó `vision` acha as bananas na imagem e publica no mesmo tópico, no mesmo formato. O picker não sabe a diferença.

```bash
ros2 launch picking_robot picking_robot.launch.py vision:=true
rqt_image_view /bananas/debug      # o que a visão está vendo, com as detecções marcadas
```

### Como uma imagem vira a posição de uma banana

1. **Amarelo.** A imagem RGB vai para HSV e sai uma máscara do amarelo da banana. O braço do robô é laranja e fica de fora da faixa.
2. **Borrões.** Cada mancha ligada é uma banana. As que encostam na borda da imagem são descartadas: estão cortadas, e de um pedaço não dá para dizer onde fica o meio.
3. **Para o espaço.** Cada pixel do borrão vira um ponto 3D com a profundidade do Kinect e os parâmetros da câmera, e a TF `map → kinect_optical` leva os pontos para o mapa.
4. **O meio do corpo.** A banana é um arco. O centro de massa dos pontos cai para o lado côncavo, onde não há banana nenhuma, e é lá que a garra desceria. Então o `detect.py` mede o **eixo médio**: fatia o borrão ao longo do comprimento e toma o centro de cada fatia. O meio desse eixo é onde a garra deve descer, e a direção dele ali é como a banana está deitada.

### O que o robô já viu

A câmera só olha para a frente, então o nó `vision` **guarda** as bananas que já encontrou, no frame `map`:

- uma detecção perto de uma banana conhecida atualiza aquela banana (média com o que já se sabia);
- uma detecção longe de todas vira uma banana nova, anunciada depois de `min_hits` quadros seguidos, para que um borrão de um quadro só não vire banana;
- uma banana conhecida que **devia** aparecer no quadro (está no campo de visão, perto o bastante, e caberia inteira na imagem) e não aparece ganha uma falta. Depois de `max_misses` faltas, é esquecida. É assim que somem as que o robô acabou de pegar.

Dois casos são tratados à parte:

- **o braço na frente:** na pose de pegar, o braço desce bem diante do Kinect e tapa quase toda a imagem. Esses quadros são descartados (o nó olha as juntas em `/joint_states`);
- **o cesto cheio:** de perto, a pilha de bananas dentro dele é um borrão amarelo grande. Um borrão cujos pontos caem dentro do círculo do cesto é ignorado — o teste é feito nos pontos, não no meio estimado, que de um monte de bananas juntas cai em qualquer lugar.

### Precisão medida

Nove bananas espalhadas de 0,3 a 2,4 m do robô, comparando a estimativa com a posição real da simulação:

| Distância | Erro de posição | Erro de rumo |
| --- | --- | --- |
| até 1,2 m | 1,1 cm (máx 1,1) | 2,7° (máx 4,6) |
| todas | 2,1 cm (máx 6,9) | 15° (máx 64) |

O que conta é a coluna de perto: o robô chega a 0,65 m da banana e estima de novo antes de descer a garra, e a tolerância da garra é de cerca de 2,5 cm. De longe a estimativa é pior, e serve só para decidir para onde ir. Uma banana apontada quase na direção da câmera fica curta na imagem, e é daí que vem o pior erro de rumo.

### Procurar

Com o Kinect, não ver nenhuma banana só quer dizer que o robô não está olhando para lá. Quando não há nenhuma banana conhecida, ele dá uma volta completa onde está e, se ainda assim não achar nada, visita os pontos de `search_points` (seis lugares livres que cobrem a casa mapeada), girando em cada um. Só depois de passar por todos sem ver nada ele dá a tarefa por terminada.

Com as posições vindas da simulação isso não acontece: ali a lista vazia já quer dizer que acabou, e o robô para assim que entrega a última (parâmetro `search` do picker, que o launch liga junto com `vision`).

### Custo

Renderizar o Kinect custa cerca de 35 ms por quadro e segura a simulação. A ponte só renderiza enquanto alguém estiver assinando as imagens, e a até `kinect_rate` quadros por segundo de simulação (5 por padrão). Sem ninguém ouvindo, o Kinect não custa nada.

## A cena

`simulation/scene_house_arm.ttt` é gerada a partir de `simulation/scene_house.ttt`, que não muda, pelo `build_scene`:

1. tira o laser (`/myRobot/LaserScanner_2D`);
2. carrega o modelo *KUKA YouBot* do CoppeliaSim e solta dele só o braço com a garra (de `youBotArmJoint0` em diante), na mesma orientação em relação à frente que ele tinha no youBot. O braço vai no topo do robô, no lugar do laser, sobre um cilindro preto que faz o papel da base dele. O resto do youBot é descartado;
3. deixa o braço **sem peso**: as peças ficam estáticas e sem colisão, e as juntas, em modo cinemático. O braço segue os ângulos comandados, mas não faz força sobre o robô, que anda como o aspirador original;
4. troca o script da garra por um que abre e fecha sob comando e que pega e solta as bananas (veja acima);
5. cria o script `/myRobot/rosInterface`, por onde a ponte lê e comanda tudo numa chamada só;
6. põe o cesto (`/bananaBasket`) no chão da sala: o *largeBasket* do CoppeliaSim, reduzido para 0,44 m de diâmetro e 0,10 m de altura, azul e fixo no lugar. Fica em (-0,53; -2,00) no frame `map`, a área mais livre da sala;
7. tira das máscaras de colisão do robô o bit usado pelas bananas;
8. põe o **Kinect** da biblioteca do CoppeliaSim (`kinect.ttm`) em `/myRobot/kinect`: com o pé encostado na frente do topo do robô, virado para a frente e inclinado 25° para baixo pela junta do próprio Kinect (`KINECT_TILT_DEG`). Com essa inclinação, ele vê o chão de ~0,2 m à frente do robô até perto do horizonte. Sem peso e sem colisão, como o braço. A barra tem as medidas reais, 28 cm, e sobra para os lados do robô, que tem 15 cm de largura.

Para o Kinect caber, a base do braço fica 5 cm atrás do centro do robô (`arm.MOUNT_Y`). Com isso, a garra alcança o chão até 25 cm à frente do centro, e é a essa distância que o robô pega as bananas.

Para gerar a cena de novo (por exemplo, depois de mudar `build_scene.py`), com um CoppeliaSim aberto. A cena aberta nele é substituída:

```bash
ros2 run picking_robot build_scene            # Porta 23000; --port para outra
```

## Navegação

O Nav2 usa `config/nav2_params.yaml`, uma cópia dos parâmetros do `vacuum_cleaner` com quatro mudanças:

- o controlador **Regulated Pure Pursuit** no lugar do DWB. O DWB cortava as quinas das portas e do cesto e entrava na zona proibida do costmap, onde travava. A simulação sem laser roda 3 a 5 vezes mais rápido que o tempo real, enquanto o controlador do Nav2 roda a 20 Hz no relógio do computador; no tempo da simulação, ele corrige o rumo poucas vezes por segundo. O RPP segue o caminho planejado de perto, gira no lugar antes de partir e freia perto de obstáculos, e por isso tolera bem essa taxa;
- o mapa também no costmap local. A cena não tem laser (a ponte publica um `/scan` vazio só para o Nav2 não reclamar), e sem o mapa o controlador não via as paredes e cortava as quinas das portas. Num teste, o centro do robô chegou a 2,5 cm de uma parede;
- `robot_radius` 0,17 m, a metade da diagonal do corpo;
- inflação de 0,45 m, fator 4, para os caminhos passarem pelo meio das portas.

A TF `map → odom` é a identidade: a odometria é a pose exata da cena, e o frame `map` é a pose inicial do robô, como no `vacuum_cleaner`.

## Tópicos e serviços

| Nome | Tipo | O que é |
| --- | --- | --- |
| `/bananas` | `geometry_msgs/PoseArray`, frame `map` | Bananas no chão: `position` é o meio da banana; `orientation` é a rotação em torno de z que alinha o eixo x ao comprimento dela. Publicado pelo `world` ou pelo `vision` |
| `/bananas/debug` | `sensor_msgs/Image` | A imagem do Kinect com as detecções marcadas (só com `vision:=true`, e só enquanto alguém assina) |
| `/myRobot/kinect/rgb/image_raw` | `sensor_msgs/Image` (`rgb8`) | Cor, 640×480 |
| `/myRobot/kinect/depth/image_raw` | `sensor_msgs/Image` (`32FC1`) | Profundidade em metros |
| `/myRobot/kinect/camera_info` | `sensor_msgs/CameraInfo` | Parâmetros da câmera, e a TF `base_link → kinect_optical` |
| `/basket` | `geometry_msgs/PoseStamped` | Centro do cesto, no `map` |
| `/bananas/delivered` | `std_msgs/Int32` | Bananas no cesto |
| `/picking/state` | `std_msgs/String` | O que o picker está fazendo |
| `/picking/start`, `/picking/stop` | `std_srvs/Trigger` | Começa e interrompe o recolhimento |
| `/myRobot/cmd_vel` | `geometry_msgs/Twist` | Velocidade da base. Sem comando por 0,6 s de simulação, as rodas param |
| `/myRobot/arm/command` | `std_msgs/Float64MultiArray` | Alvo das 5 juntas do braço [rad], preso aos limites |
| `/myRobot/arm/pose` | `std_msgs/String` | Pose pronta: `repouso`, `alto`, `chao` ou `carregar` |
| `/myRobot/gripper/command` | `std_msgs/Bool` | `true` fecha a garra, `false` abre |
| `/myRobot/gripper/holding` | `std_msgs/Bool` | Se a garra segura uma banana |
| `/joint_states` | `sensor_msgs/JointState` | As 5 juntas do braço [rad] e os 2 dedos [m] |
| `/myRobot/odom` e TF `odom → base_link` | `nav_msgs/Odometry` | Pose exata da simulação, com a origem onde o robô começou |
| `/map` | `nav_msgs/OccupancyGrid` | Mapa da casa com o cesto |
| `/scan` | `sensor_msgs/LaserScan` | Vazio (a cena não tem laser) |
| `/clock` | `rosgraph_msgs/Clock` | Tempo da simulação |

## O braço

`picking_robot/arm.py` tem os nomes, os limites e as poses, além da cinemática no plano do braço, medida na cena:

- `fingertip(q)` diz onde fica a ponta dos dedos, no referencial do robô (y para a frente, z para cima);
- `reach(y, z)` devolve as juntas para a ponta ir a um ponto, com a garra apontada para baixo;
- `reach_any` testa inclinações da garra quando apontar para baixo não alcança, como ao soltar a banana no cesto.

A pose `chao` deixa a ponta dos dedos 2 cm acima do chão, 25 cm à frente do centro do robô. Na simulação, ela ficou a 6 mm do previsto.

## Testes

Rodam sem ROS e sem simulador (braço, geometria de pegar e de chegar):

```bash
cd ros_ws/src/picking_robot && PYTHONPATH=. python3 -m pytest test
```

## Limitações

- **Física da pega:** a garra não pega nada pela física; o script da cena prende a banana à mão. Ela não escorrega nem cai no caminho.
- **O braço atravessa o que estiver no caminho**, inclusive o chão e o próprio robô. As poses prontas evitam isso; os comandos junta a junta, não.
- **Bananas perto de paredes:** o sorteio as deixa a 0,45 m ou mais das paredes. Uma banana mais perto, posta à mão, pode não ter lado livre para o robô chegar, e ele desiste dela.
- **Obstáculos novos:** sem laser, o Nav2 só conhece o mapa e o cesto. Um objeto novo no caminho não é visto — nem pelo Kinect, que só é usado para achar bananas.
- **Qualquer amarelo é banana:** a visão procura cor, não forma. Numa cena com outros objetos amarelos no chão, o robô tentaria pegá-los. Trocar o `detect.py` por um detector treinado (YOLO, como no `potential_field`) resolveria, sem mexer no resto.
- **A câmera não vê o que está logo à frente:** o Kinect enxerga do chão a partir de ~0,2 m à frente do robô. Na hora de fechar a garra, a banana já saiu por baixo da imagem; o robô usa a última estimativa e a odometria, que na simulação é exata.
- **Cesto cheio:** com muitas bananas, elas se empilham acima da borda e podem cair para fora. Uma que caia fora volta a ser recolhida.
- **O robô escorrega devagar quando parado** (~17 mm em 20 s de simulação). A cena original faz o mesmo, então não é efeito do braço.
