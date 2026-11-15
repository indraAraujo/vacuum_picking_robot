"""Cômodos da cena simulation/scene_house.ttt: onde ficam e para onde ir.

Fonte única dos lugares que o robô conhece por nome. Daqui saem os destinos
mandados ao Nav2 e a resposta de "em que cômodo estou".

Referenciais
------------
As coordenadas abaixo estão no referencial **da cena** (mundo do CoppeliaSim),
medidas a partir das paredes, portas e móveis de ``/house``. O Nav2 trabalha
no frame ``map``, que o SLAM ancora na pose inicial do robô:

    map = cena - SCENE_ORIGIN       (sem rotação: o robô começa olhando para +X)

A pose inicial do robô na cena é (3,55; 0,775), com a frente para +X. Se o
robô for movido na cena antes de subir o projeto, o frame ``map`` muda junto e
SCENE_ORIGIN precisa ser atualizado.

Cada cômodo tem:

- ``goal``: ponto livre no meio do cômodo, longe de paredes e móveis, que vira
  o destino do Nav2;
- ``views``: pontos extras de onde olhar ao mapear, além do destino. O laser
  não atravessa móveis: num cômodo grande, girar de um ponto só deixa a cama
  ou o sofá cobrindo tudo o que está atrás. Cômodo pequeno não precisa de
  nenhum;
- ``areas``: retângulos ``(x_min, x_max, y_min, y_max)`` usados para dizer em
  que cômodo o robô está e para prender a limpeza ao cômodo: fora deles, o
  cleaning_server enxerga uma parede virtual. Cômodos em L, como o corredor,
  usam mais de um retângulo. As bordas seguem as paredes, e nas portas e
  passagens abertas ficam na linha da parede.
"""

import math

#: Pose inicial do robô na cena = origem do frame map.
SCENE_ORIGIN = (3.55, 0.775)


class Room:
    """Um cômodo: nome falado, artigo, destino e área (na cena)."""

    def __init__(self, name, article, goal, areas, views=()):
        self.name = name
        self.article = article
        self.goal_scene = goal
        self.areas_scene = areas
        self.views_scene = tuple(views)

    @property
    def goal(self):
        """Destino no frame map."""
        return to_map(*self.goal_scene)

    @property
    def views(self):
        """Pontos extras de observação, no frame map."""
        return [to_map(*p) for p in self.views_scene]

    @property
    def area_flat(self):
        """Retângulos no frame map, em lista: [x_min, x_max, y_min, y_max, ...]."""
        flat = []
        for x0, x1, y0, y1 in self.areas_scene:
            (a, b), (c, d) = to_map(x0, y0), to_map(x1, y1)
            flat += [a, c, b, d]
        return flat

    def contains_scene(self, x, y):
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, x1, y0, y1 in self.areas_scene)


#: Os cômodos, na ordem em que o robô percorre a casa ("limpar tudo"): os sete
#: do interior, do oeste para o leste, terminando perto da base.
#:
#: A entrada fica de fora de propósito. Ela não é um cômodo: passada a porta da
#: frente, o chão é a calçada do lado de fora, mais baixa que o piso da casa e
#: ligada a ele por uma rampa. O aspirador não desce lá.
ROOMS = (
    Room('sala de jantar', 'a', (2.80, 0.30), [(-0.10, 4.40, -0.10, 1.40)]),
    Room('cozinha', 'a', (3.20, 4.20), [(-0.10, 4.40, 1.40, 5.50)],
         views=[(0.90, 4.60), (0.80, 2.20)]),
    Room('sala', 'a', (3.00, -1.60), [(-0.10, 4.40, -3.70, -0.10)],
         views=[(1.20, -0.80), (3.40, -2.60)]),
    Room('lavabo', 'o', (5.15, 3.70), [(4.40, 5.90, 3.00, 5.50)]),
    # O corredor é um T deitado: a faixa leste-oeste até a porta da frente, o
    # trecho em frente ao lavabo e o trecho em frente ao banheiro (onde fica o
    # armário do quarto, encostado na parede do fundo).
    Room('corredor', 'o', (6.20, 0.70), [(4.40, 8.90, -0.10, 1.40),
                                         (4.40, 6.00, 1.40, 3.00),
                                         (6.00, 8.70, 1.40, 3.20)]),
    # O banheiro é um L: a antessala estreita, de 0,70 m, e o cômodo em si.
    Room('banheiro', 'o', (7.30, 4.60), [(7.95, 8.65, 2.80, 3.80),
                                         (6.05, 8.65, 3.80, 5.45)],
         views=[(6.60, 4.60)]),
    # A cama ocupa o meio do quarto: de um ponto só, o laser mapeia metade.
    Room('quarto', 'o', (9.60, 2.80), [(8.75, 12.55, 1.40, 5.45)],
         views=[(11.90, 4.90), (12.00, 3.00)]),
)

BY_NAME = {room.name: room for room in ROOMS}

#: A base é onde o robô começou: a origem do frame map.
BASE = (0.0, 0.0)


def to_map(x, y):
    """Ponto da cena -> frame map."""
    return x - SCENE_ORIGIN[0], y - SCENE_ORIGIN[1]


def to_scene(x, y):
    """Ponto do frame map -> cena."""
    return x + SCENE_ORIGIN[0], y + SCENE_ORIGIN[1]


def room_names():
    return [room.name for room in ROOMS]


def room_at(x, y):
    """Nome do cômodo que contém o ponto (frame map), ou None."""
    sx, sy = to_scene(x, y)
    for room in ROOMS:
        if room.contains_scene(sx, sy):
            return room.name
    return None


def heading(from_xy, to_xy):
    """Ângulo de quem vai de from_xy até to_xy, para orientar o destino."""
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])
