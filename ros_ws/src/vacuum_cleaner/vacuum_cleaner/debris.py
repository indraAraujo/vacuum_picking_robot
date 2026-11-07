"""Onde espalhar a sujeira: posições sorteadas no chão livre do mapa.

Sem ROS e sem simulador, para dar para testar. O debris_server usa isto para
escolher onde soltar as bananas na cena.

Regras do sorteio, todas em metros no referencial ``map``:

- longe das paredes (``clearance``): a varredura mantém o robô a ~0,28 m das
  paredes, e ele recolhe o que estiver a até 0,15 m do centro. Uma banana mais
  perto da parede do que isso nunca seria recolhida;
- alcançável a partir da base: o ponto precisa estar ligado à posição inicial
  do robô por um caminho com folga para o corpo dele (``reach_clearance``).
  Isso descarta os vãos fechados que o mapa às vezes tem, como o espaço sob um
  sofá ou atrás de um armário;
- nem em cima da base (``base_clearance``), nem colada em outra banana
  (``spacing``).
"""

import math
import os
import random
from collections import deque


def read_pgm(path):
    """Lê um PGM binário (P5) de 8 bits: (largura, altura, pixels)."""
    with open(path, 'rb') as f:
        data = f.read()
    campos, pos = [], 0
    while len(campos) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        fim = pos
        while not data[fim:fim + 1].isspace():
            fim += 1
        campos.append(data[pos:fim])
        pos = fim
    if campos[0] != b'P5' or int(campos[3]) > 255:
        raise ValueError(f'{path}: só PGM binário de 8 bits (P5)')
    largura, altura = int(campos[1]), int(campos[2])
    pixels = data[pos + 1:pos + 1 + largura * altura]
    return largura, altura, pixels


def read_map_yaml(path):
    """Lê o YAML do map_server sem depender do PyYAML (só chave: valor)."""
    info = {}
    with open(path) as f:
        for linha in f:
            linha = linha.split('#', 1)[0].strip()
            if ':' not in linha:
                continue
            chave, valor = (s.strip() for s in linha.split(':', 1))
            info[chave] = valor
    origem = [float(v) for v in info['origin'].strip('[]').split(',')]
    imagem = info['image']
    if not os.path.isabs(imagem):
        imagem = os.path.join(os.path.dirname(path), imagem)
    return {
        'image': imagem,
        'resolution': float(info['resolution']),
        'origin': origem,
        'negate': int(info.get('negate', 0)),
        'free_thresh': float(info.get('free_thresh', 0.196)),
    }


class FloorMap:
    """Células livres do mapa e a distância de cada uma ao obstáculo mais perto."""

    def __init__(self, width, height, resolution, origin_x, origin_y, free):
        # ``free`` é uma lista por linha, com a linha 0 em y = origin_y (embaixo).
        self.width, self.height = width, height
        self.res = resolution
        self.ox, self.oy = origin_x, origin_y
        self.free = free
        self.dist = self._distance()

    @classmethod
    def from_yaml(cls, path):
        info = read_map_yaml(path)
        w, h, px = read_pgm(info['image'])
        livre = []
        for j in range(h):              # j = 0 é a última linha da imagem
            linha = px[(h - 1 - j) * w:(h - j) * w]
            ocup = [(v / 255.0) if info['negate'] else (255 - v) / 255.0 for v in linha]
            livre.append([o < info['free_thresh'] for o in ocup])
        ox, oy = info['origin'][0], info['origin'][1]
        return cls(w, h, info['resolution'], ox, oy, livre)

    def _distance(self):
        """Distância (em metros) até a célula não livre mais próxima (chamfer 3-4)."""
        inf = 10 ** 9
        w, h = self.width, self.height
        d = [[0 if not self.free[j][i] else inf for i in range(w)] for j in range(h)]
        passos = ((-1, -1, 4), (0, -1, 3), (1, -1, 4), (-1, 0, 3))
        for j in range(h):
            for i in range(w):
                if d[j][i]:
                    for di, dj, c in passos:
                        a, b = i + di, j + dj
                        v = d[b][a] + c if 0 <= a < w and 0 <= b < h else c
                        if v < d[j][i]:
                            d[j][i] = v
        for j in range(h - 1, -1, -1):
            for i in range(w - 1, -1, -1):
                if d[j][i]:
                    for di, dj, c in passos:
                        a, b = i - di, j - dj
                        v = d[b][a] + c if 0 <= a < w and 0 <= b < h else c
                        if v < d[j][i]:
                            d[j][i] = v
        # Entre centros de célula; menos meia célula, até a borda do obstáculo.
        return [[max(0.0, v * self.res / 3.0 - self.res / 2.0) if v else 0.0
                 for v in linha] for linha in d]

    def cell(self, x, y):
        return math.floor((x - self.ox) / self.res), math.floor((y - self.oy) / self.res)

    def center(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def clearance(self, x, y):
        i, j = self.cell(x, y)
        if not (0 <= i < self.width and 0 <= j < self.height):
            return 0.0
        return self.dist[j][i]

    def reachable(self, start, min_clearance):
        """Células ligadas a ``start`` por células com folga >= min_clearance."""
        i0, j0 = self.cell(*start)
        ok = (lambda i, j: 0 <= i < self.width and 0 <= j < self.height
              and self.dist[j][i] >= min_clearance)
        if not ok(i0, j0):
            # A base costuma ficar rente à parede: começa da célula com folga
            # mais próxima.
            candidatas = [(abs(i - i0) + abs(j - j0), i, j)
                          for j in range(max(0, j0 - 10), min(self.height, j0 + 11))
                          for i in range(max(0, i0 - 10), min(self.width, i0 + 11))
                          if ok(i, j)]
            if not candidatas:
                return set()
            _, i0, j0 = min(candidatas)
        visto = {(i0, j0)}
        fila = deque(visto)
        while fila:
            i, j = fila.popleft()
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if (a, b) not in visto and ok(a, b):
                    visto.add((a, b))
                    fila.append((a, b))
        return visto


def sample_spots(floor, count, start=(0.0, 0.0), clearance=0.30,
                 reach_clearance=0.20, base_clearance=0.6, spacing=0.4, rng=None):
    """Sorteia até ``count`` pontos (x, y) no chão livre, alcançável e longe das paredes."""
    rng = rng or random.Random()
    celulas = [c for c in floor.reachable(start, reach_clearance)
               if floor.dist[c[1]][c[0]] >= clearance]
    rng.shuffle(celulas)
    pontos = []
    for i, j in celulas:
        if len(pontos) >= count:
            break
        x, y = floor.center(i, j)
        x += rng.uniform(-0.4, 0.4) * floor.res
        y += rng.uniform(-0.4, 0.4) * floor.res
        if math.hypot(x - start[0], y - start[1]) < base_clearance:
            continue
        if any(math.hypot(x - px, y - py) < spacing for px, py in pontos):
            continue
        pontos.append((x, y))
    return pontos
