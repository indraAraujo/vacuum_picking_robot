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
