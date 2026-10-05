"""Cliente minimo de KUKAVARPROXY (mismo protocolo que py-openshowvar).

A diferencia de py-openshowvar: timeouts en el socket, lectura completa del
mensaje segun su cabecera, reconexion automatica y errores como excepciones.
No es thread-safe: protegerlo con un lock desde fuera.
"""

import re
import socket
import struct


class KvpError(Exception):
    pass


class KvpClient:
    def __init__(self, ip, port=7000, timeout=2.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self._sock = None
        self._msg_id = 1

    # --- conexion -------------------------------------------------------
    def connect(self):
        self.close()
        sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def connected(self):
        return self._sock is not None

    # --- API ------------------------------------------------------------
    def read(self, var):
        return self._request(var, None)

    def write(self, var, value):
        return self._request(var, value)

    # --- protocolo ------------------------------------------------------
    def _request(self, var, value):
        """Envia una peticion; si falla el socket reconecta y reintenta una vez."""
        for attempt in (0, 1):
            try:
                if self._sock is None:
                    self.connect()
                return self._transact(var, value)
            except (OSError, KvpError) as e:
                self.close()
                if attempt == 1 or isinstance(e, KvpError):
                    raise KvpError(f'{var}: {e}') from e

    def _transact(self, var, value):
        name = var.encode()
        if value is None:
            body = struct.pack(f'!BH{len(name)}s', 0, len(name), name)
        else:
            val = value.encode()
            body = struct.pack(f'!BH{len(name)}sH{len(val)}s',
                               1, len(name), name, len(val), val)
        msg_id = self._msg_id
        self._msg_id = (self._msg_id + 1) % 65536
        self._sock.sendall(struct.pack('!HH', msg_id, len(body)) + body)

        rsp_id, rsp_len = struct.unpack('!HH', self._recv_exact(4))
        rsp = self._recv_exact(rsp_len)
        if rsp_id != msg_id:
            raise OSError(f'msg_id desincronizado ({rsp_id} != {msg_id})')
        # rsp = flag(1) + len(2) + valor + estado(3); ultimo byte 0x01 = OK
        val_len = struct.unpack('!H', rsp[1:3])[0]
        data = rsp[3:3 + val_len]
        if not rsp.endswith(b'\x01'):
            raise KvpError(f'el controlador rechazo la peticion (respuesta {data!r})')
        return data

    def _recv_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise OSError('conexion cerrada por el controlador')
            buf += chunk
        return buf


_AXIS_RE = re.compile(r'\bA([1-6])\s+(-?[\d.]+(?:[eE][-+]?\d+)?)')


def parse_axes(raw):
    """b'{E6AXIS: A1 0.0, A2 -90.0, ...}' -> [A1..A6] en grados."""
    found = {int(i): float(v) for i, v in _AXIS_RE.findall(raw.decode(errors='replace'))}
    if sorted(found) != [1, 2, 3, 4, 5, 6]:
        raise KvpError(f'no se pudo parsear ejes de {raw!r}')
    return [found[i] for i in range(1, 7)]


def format_e6axis(deg):
    """[A1..A6] en grados -> literal KRL E6AXIS con E1..E6 = 0.0."""
    axes = ', '.join(f'A{i + 1} {v:.4f}' for i, v in enumerate(deg))
    return '{E6AXIS: ' + axes + ', E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}'
