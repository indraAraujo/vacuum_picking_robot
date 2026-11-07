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
- ``areas``: retângulos ``(x_min, x_max, y_min, y_max)`` usados para dizer em
  que cômodo o robô está. Cômodos em L, como o corredor, usam mais
  de um retângulo.
"""

import math

#: Pose inicial do robô na cena = origem do frame map.
SCENE_ORIGIN = (3.55, 0.775)


class Room:
    """Um cômodo: nome falado, artigo, destino e área (na cena)."""

    def __init__(self, name, article, goal, areas):
        self.name = name
        self.article = article
        self.goal_scene = goal
        self.areas_scene = areas

    @property
    def goal(self):
        """Destino no frame map."""
        return to_map(*self.goal_scene)

    def contains_scene(self, x, y):
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, x1, y0, y1 in self.areas_scene)


#: Os cômodos, na ordem em que o robô percorre a casa ("limpar tudo").
#:
#: Entrada, banheiro e quarto ficam de fora: na cena eles estão atrás de portas
#: fechadas, o laser vê as portas como parede e esses cômodos não entram no
#: mapa. Para devolvê-los, abra as portas na cena, remapeie e acrescente aqui:
#:   Room('entrada', 'a', (6.70, -0.80), [(4.40, 8.90, -1.70, -0.10)])
#:   Room('banheiro', 'o', (7.30, 4.50), [(6.00, 8.90, 3.50, 5.50)])
#:   Room('quarto', 'o', (9.70, 2.80), [(6.00, 12.60, 1.30, 3.50),
#:                                      (8.90, 12.60, 3.50, 5.50)])
ROOMS = (
    Room('sala de jantar', 'a', (2.80, 0.30), [(-0.10, 4.40, -0.10, 1.40)]),
    Room('cozinha', 'a', (3.20, 4.20), [(-0.10, 4.40, 1.40, 5.50)]),
    Room('sala', 'a', (3.00, -1.60), [(-0.10, 4.40, -3.70, -0.10)]),
    Room('lavabo', 'o', (5.15, 3.70), [(4.40, 5.90, 3.00, 5.50)]),
    # O corredor também: a faixa leste-oeste e o trecho em frente ao lavabo.
    Room('corredor', 'o', (6.20, 0.70), [(4.40, 12.60, -0.10, 1.30),
                                          (4.40, 6.00, 1.30, 3.00)]),
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
