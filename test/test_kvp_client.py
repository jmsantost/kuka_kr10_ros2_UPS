import pytest

from kuka_bridge.kvp_client import KvpClient, KvpError, format_e6axis, parse_axes

from conftest import free_port


def test_read_write_roundtrip(make_sim):
    sim = make_sim()
    c = KvpClient('127.0.0.1', sim.port)
    try:
        assert c.read('COM_WR') == b'0'
        c.write('COM_LAST', '42')
        assert c.read('COM_LAST') == b'42'
        c.write('COM_BUF[3]', format_e6axis([1, 2, 3, 4, 5, 6]))
        assert parse_axes(c.read('COM_BUF[3]')) == [1, 2, 3, 4, 5, 6]
        assert c.read('$MODE_OP') == b'#T1'
    finally:
        c.close()


def test_many_requests_keep_message_ids_in_sync(make_sim):
    sim = make_sim()
    c = KvpClient('127.0.0.1', sim.port)
    try:
        for i in range(300):
            c.write('COM_LAST', str(i))
            assert c.read('COM_LAST') == str(i).encode()
    finally:
        c.close()


def test_reconnects_after_server_restart(make_sim):
    sim = make_sim()
    c = KvpClient('127.0.0.1', sim.port)
    try:
        assert c.read('COM_WR') == b'0'
        sim.stop()
        sim.start()   # mismo puerto, conexion anterior rota
        assert c.read('COM_WR') == b'0'
    finally:
        c.close()


def test_unreachable_raises_kvp_error():
    c = KvpClient('127.0.0.1', free_port(), timeout=0.5)
    with pytest.raises(KvpError):
        c.read('$AXIS_ACT')


def test_parse_axes_from_controller_format():
    raw = (b'{E6AXIS: A1 7.72200, A2 -89.9944, A3 89.9995, A4 0.00380000, '
           b'A5 4.00000E-04, A6 -1.96709620E-32, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}')
    a = parse_axes(raw)
    assert a[0] == pytest.approx(7.722)
    assert a[4] == pytest.approx(4e-4)
    assert a[5] == pytest.approx(0.0)


def test_parse_axes_rejects_incomplete():
    with pytest.raises(KvpError):
        parse_axes(b'{E6AXIS: }')


def test_format_e6axis():
    s = format_e6axis([0, -90, 90.123456, 0, 0, 0])
    assert s.startswith('{E6AXIS: A1 0.0000, A2 -90.0000, A3 90.1235,')
    assert s.endswith('E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}')
