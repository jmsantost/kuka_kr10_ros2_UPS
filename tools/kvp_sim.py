#!/usr/bin/env python3
"""Simulador minimo de KUKAVARPROXY + programa KRL, para probar kuka_bridge sin robot.

Habla el mismo protocolo TCP que KUKAVARPROXY y emula uno de los dos programas:
  - ros_server.src (PROG=ROS_SERVER): COM_ACTION=1 -> "mueve" a COM_E6AXIS -> COM_ACTION=0
  - ros_stream.src (PROG=ROS_STREAM): lee COM_BUF[] mientras COM_RD < COM_WR, planifica
    hasta 3 puntos por delante ($ADVANCE=3) y tarda STEP_S segundos por punto.

Variables de entorno:
  PROG=ROS_SERVER|ROS_STREAM   programa "seleccionado"        (defecto ROS_SERVER)
  OPMODE=T1|T2|AUT|EX          $MODE_OP                        (defecto T1)
  STEP_S=0.3                   duracion simulada de cada PTP
  PORT=17000                   puerto TCP (escucha solo en 127.0.0.1)

IMPORTANTE: probar en un dominio ROS aislado para que los goals de prueba no
lleguen a un bridge conectado al robot real:

  export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  PROG=ROS_STREAM python3 tools/kvp_sim.py &
  ros2 launch kuka_bridge moveit.launch.py robot_ip:=127.0.0.1 robot_port:=17000 \\
      allow_motion:=true protocol:=stream
"""

import os
import socket
import struct
import threading
import time

PROG = os.environ.get('PROG', 'ROS_SERVER').upper()
STEP_S = float(os.environ.get('STEP_S', '0.3'))
PORT = int(os.environ.get('PORT', '17000'))

V = {
    '$AXIS_ACT': '{E6AXIS: A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0, '
                 'E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}',
    '$MODE_OP': '#' + os.environ.get('OPMODE', 'T1'),
    '$PRO_NAME1[]': f'"{PROG}"',
    '$PRO_STATE1': '#P_ACTIVE',
    '$OV_PRO': '10',
    'COM_ACTION': '0',
    'COM_WR': '0', 'COM_RD': '0', 'COM_LAST': '0', 'COM_GEN': '0',
}
SOFT_LIMITS = [(-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350)]
for i, (lo, hi) in enumerate(SOFT_LIMITS, start=1):
    V[f'$SOFTN_END[{i}]'] = str(lo)
    V[f'$SOFTP_END[{i}]'] = str(hi)


def ros_server():
    while True:
        if V['COM_ACTION'] == '1':
            target = V['COM_E6AXIS']
            time.sleep(STEP_S)
            V['$AXIS_ACT'] = target
            V['COM_ACTION'] = '0'
            print('PTP ->', target, flush=True)
        time.sleep(0.02)


def ros_stream():
    V['COM_RD'] = V['COM_WR']
    V['COM_GEN'] = str(int(V['COM_GEN']) + 1)
    planned = []   # (seq, target, exacto)
    while True:
        rd, wr = int(V['COM_RD']), int(V['COM_WR'])
        if len(planned) < 3 and rd < wr:          # advance run
            seq = rd + 1
            planned.append((seq, V.get(f'COM_BUF[{rd % 16 + 1}]', ''),
                            seq == int(V['COM_LAST'])))
            V['COM_RD'] = str(seq)
        if planned:                               # main run
            seq, target, exact = planned.pop(0)
            time.sleep(STEP_S)
            V['$AXIS_ACT'] = target
            print(f'seq {seq} {"PTP" if exact else "PTP C_PTP"} -> {target}', flush=True)
        time.sleep(0.01)


def recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError
        buf += chunk
    return buf


def handle(conn):
    try:
        while True:
            msg_id, length = struct.unpack('!HH', recv_exact(conn, 4))
            body = recv_exact(conn, length)
            flag, name_len = struct.unpack('!BH', body[:3])
            name = body[3:3 + name_len].decode()
            if flag == 1:   # escritura
                val_len = struct.unpack('!H', body[3 + name_len:5 + name_len])[0]
                V[name] = body[5 + name_len:5 + name_len + val_len].decode()
            value = V.get(name, '').encode()
            rsp = struct.pack('!BH', flag, len(value)) + value + b'\x00\x01\x01'
            conn.sendall(struct.pack('!HH', msg_id, len(rsp)) + rsp)
    except (ConnectionError, OSError):
        conn.close()


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', PORT))
    srv.listen()
    print(f'kvp_sim en 127.0.0.1:{PORT} | {PROG} | {V["$MODE_OP"]}', flush=True)
    threading.Thread(target=ros_stream if PROG == 'ROS_STREAM' else ros_server,
                     daemon=True).start()
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle, args=(conn,), daemon=True).start()


if __name__ == '__main__':
    main()
