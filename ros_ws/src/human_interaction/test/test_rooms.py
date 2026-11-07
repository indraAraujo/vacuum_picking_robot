"""Testes da tabela de cômodos e da ligação dela com o vocabulário.

    cd ros_ws/src/human_interaction && PYTHONPATH=. python3 -m pytest test/
"""

import pytest

from human_interaction import commands, rooms


@pytest.mark.parametrize('room', rooms.ROOMS, ids=lambda r: r.name)
def test_destino_cai_no_proprio_comodo(room):
    assert rooms.room_at(*room.goal) == room.name


def test_base_fica_na_sala_de_jantar():
    assert rooms.room_at(*rooms.BASE) == 'sala de jantar'


def test_conversao_de_referencial_e_inversa():
    assert rooms.to_scene(*rooms.to_map(6.2, 0.7)) == pytest.approx((6.2, 0.7))


def test_todo_lugar_do_vocabulario_e_um_comodo():
    for nome in commands.PLACES.values():
        assert nome == 'casa toda' or nome in rooms.BY_NAME


@pytest.mark.parametrize('room', rooms.ROOMS, ids=lambda r: r.name)
def test_todo_comodo_pode_ser_pedido(room):
    for frase in (f'vá para {room.article} {room.name}',
                  f'limpar {room.article} {room.name}'):
        intent, args, _ = commands.parse(frase)
        assert args.get('local') == room.name, frase


def test_area_da_limpeza_no_frame_map():
    sala = rooms.BY_NAME['sala']
    flat = sala.area_flat
    assert len(flat) == 4 * len(sala.areas_scene)
    x0, x1, y0, y1 = flat[:4]
    assert x0 < x1 and y0 < y1
    # O destino do Nav2 fica dentro da área que prende a varredura.
    gx, gy = sala.goal
    assert x0 <= gx <= x1 and y0 <= gy <= y1
