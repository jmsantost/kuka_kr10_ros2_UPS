"""Fixtures comunes.

SEGURIDAD: los tests nunca deben llegar al robot real.
- Todo se conecta a 127.0.0.1 (simulador tools/kvp_sim.py).
- Se fuerza un ROS_DOMAIN_ID aleatorio con descubrimiento solo local, para que
  ningun goal de prueba llegue a un kuka_bridge real que este corriendo en la red.
"""

import os
import random
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

# Antes de importar rclpy en cualquier test
os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'tools' / 'kvp_sim.py'
sys.path.insert(0, str(ROOT))   # importar kuka_bridge sin instalar


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Sim:
    """Proceso del simulador KUKAVARPROXY + programa KRL."""

    def __init__(self, port, **env):
        self.port = port
        self.env = {'PROG': 'ROS_STREAM', 'OPMODE': 'T1', 'STEP_S': '0.05', **env}
        self.proc = None

    def start(self):
        env = {**os.environ, **{k: str(v) for k, v in self.env.items()},
               'PORT': str(self.port), 'PYTHONUNBUFFERED': '1'}
        self.proc = subprocess.Popen([sys.executable, str(SIM)], env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                socket.create_connection(('127.0.0.1', self.port), timeout=0.2).close()
                return self
            except OSError:
                time.sleep(0.05)
        self.stop()
        raise RuntimeError('el simulador no arranco')

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait(timeout=5)

    def output(self):
        self.stop()
        return self.proc.stdout.read()


@pytest.fixture
def make_sim():
    sims = []

    def _make(**env):
        sim = Sim(free_port(), **env).start()
        sims.append(sim)
        return sim
    yield _make
    for sim in sims:
        sim.stop()
