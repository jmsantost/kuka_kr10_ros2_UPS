"""Integracion: KukaBridge real contra tools/kvp_sim.py en 127.0.0.1.

Ver conftest.py: ROS_DOMAIN_ID aleatorio y descubrimiento solo local.
"""

import math
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from kuka_bridge.kuka_bridge_node import KukaBridge  # noqa: E402
from kuka_bridge.kvp_client import KvpClient, parse_axes  # noqa: E402

Result = FollowJointTrajectory.Result
JOINTS = [f'joint_{i}' for i in range(1, 7)]
HOME = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]


class Harness:
    def __init__(self, sim, **params):
        assert sim.port   # siempre el simulador local
        params = {'robot_ip': '127.0.0.1', 'robot_port': sim.port, 'allow_motion': True,
                  'protocol': 'stream', 'poll_period_s': 0.01, 'motion_timeout_s': 10.0,
                  **params}
        args = ['--ros-args']
        for k, v in params.items():
            v = str(v).lower() if isinstance(v, bool) else v
            args += ['-p', f'{k}:={v}']
        self.ctx = rclpy.Context()
        rclpy.init(args=args, context=self.ctx)
        self.bridge = KukaBridge(context=self.ctx)
        self.client_node = rclpy.create_node('test_client', context=self.ctx)
        self.client = ActionClient(self.client_node, FollowJointTrajectory,
                                   'joint_trajectory_controller/follow_joint_trajectory')
        self.executor = MultiThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.client_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_server(timeout_sec=10)
        self.sim = sim

    def wait(self, future, timeout=30):
        deadline = time.monotonic() + timeout
        while not future.done():
            assert time.monotonic() < deadline, 'timeout esperando al bridge'
            time.sleep(0.01)
        return future.result()

    def send(self, points, names=JOINTS, cancel_after=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for k, p in enumerate(points):
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=[math.radians(v) for v in p]))
            goal.trajectory.points[-1].time_from_start.sec = k + 1
        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted
        if cancel_after is not None:
            time.sleep(cancel_after)
            self.wait(handle.cancel_goal_async())
        res = self.wait(handle.get_result_async())
        return res.status, res.result

    def robot_axes(self):
        c = KvpClient('127.0.0.1', self.sim.port)
        try:
            return parse_axes(c.read('$AXIS_ACT'))
        finally:
            c.close()

    def read(self, var):
        c = KvpClient('127.0.0.1', self.sim.port)
        try:
            return c.read(var).decode()
        finally:
            c.close()

    def close(self):
        self.executor.shutdown()
        self.bridge.client.close()
        self.bridge.destroy_node()
        self.client_node.destroy_node()
        rclpy.shutdown(context=self.ctx)
        self.thread.join(timeout=5)


@pytest.fixture
def bridge(make_sim):
    harnesses = []

    def _make(sim_env=None, **params):
        h = Harness(make_sim(**(sim_env or {})), **params)
        harnesses.append(h)
        return h
    yield _make
    for h in harnesses:
        h.close()


def line(start, end, steps):
    return [[a + (b - a) * k / steps for a, b in zip(start, end)] for k in range(1, steps + 1)]


def approx_axes(actual, expected, tol=0.01):
    return all(abs(a - e) <= tol for a, e in zip(actual, expected))


# --- /joint_states -------------------------------------------------------------

def test_publishes_joint_states_in_radians(bridge):
    h = bridge()
    msgs = []
    h.client_node.create_subscription(JointState, 'joint_states', msgs.append, 10)
    deadline = time.monotonic() + 5
    while not msgs and time.monotonic() < deadline:
        time.sleep(0.05)
    assert msgs, 'no llego /joint_states'
    assert list(msgs[-1].name) == JOINTS
    assert [math.degrees(v) for v in msgs[-1].position] == pytest.approx(HOME, abs=1e-6)


# --- ejecucion -----------------------------------------------------------------

@pytest.mark.parametrize('protocol,prog', [('stream', 'ROS_STREAM'), ('single', 'ROS_SERVER')])
def test_l_trajectory_reaches_goal(bridge, protocol, prog):
    h = bridge(sim_env={'PROG': prog}, protocol=protocol)
    corner = [8, -90, 90, 0, 0, 0]
    end = [8, -82, 90, 0, 0, 0]
    status, result = h.send(line(HOME, corner, 16) + line(corner, end, 16))
    assert result.error_code == Result.SUCCESSFUL, result.error_string
    assert approx_axes(h.robot_axes(), end)


def test_stream_chains_with_c_ptp_and_stops_on_last(bridge):
    h = bridge()
    corner = [8, -90, 90, 0, 0, 0]
    end = [8, -82, 90, 0, 0, 0]
    _, result = h.send(line(HOME, corner, 16) + line(corner, end, 16))
    assert result.error_code == Result.SUCCESSFUL
    out = h.sim.output()
    assert 'seq 1 PTP C_PTP' in out    # esquina aproximada
    assert 'seq 2 PTP ->' in out       # ultimo punto exacto
    assert 'seq 3' not in out          # simplificado a 2 waypoints


def test_unordered_joint_names_are_mapped(bridge):
    h = bridge()
    target = [5, -88, 90, 0, 3, 0]
    names = ['joint_3', 'joint_1', 'joint_6', 'joint_2', 'joint_5', 'joint_4']
    pts = [[p[int(n[-1]) - 1] for n in names] for p in line(HOME, target, 4)]
    _, result = h.send(pts, names=names)
    assert result.error_code == Result.SUCCESSFUL, result.error_string
    assert approx_axes(h.robot_axes(), target)


# --- validacion: nada debe llegar al robot ---------------------------------------

def assert_rejected(h, points, code, text, names=JOINTS):
    _, result = h.send(points, names=names)
    assert result.error_code == code
    assert text in result.error_string
    assert h.read('COM_WR') == '0', 'se escribio en la cola'
    assert approx_axes(h.robot_axes(), HOME), 'el robot se movio'


def test_allow_motion_false_sends_nothing(bridge):
    h = bridge(allow_motion=False)
    assert_rejected(h, line(HOME, [5, -90, 90, 0, 0, 0], 5), Result.INVALID_GOAL, 'allow_motion')


def test_joint_limit_rejected(bridge):
    h = bridge()
    # A2 llega a +45 (limite URDF 45, con margen 44)
    assert_rejected(h, line(HOME, [0, 45, 90, 0, 0, 0], 30), Result.INVALID_GOAL, 'fuera de')


def test_jump_rejected(bridge):
    h = bridge()
    assert_rejected(h, [[20, -90, 90, 0, 0, 0]], Result.INVALID_GOAL, 'salto')


def test_max_total_rejected_when_enabled(bridge):
    h = bridge(max_total_deg=15.0)
    assert_rejected(h, line(HOME, [20, -90, 90, 0, 0, 0], 8), Result.INVALID_GOAL, 'max_total')


def test_wrong_joint_names_rejected(bridge):
    h = bridge()
    assert_rejected(h, [HOME], Result.INVALID_JOINTS, 'joints',
                    names=['a', 'b', 'c', 'd', 'e', 'f'])


def test_aut_blocked_by_default(bridge):
    h = bridge(sim_env={'OPMODE': 'AUT'})
    assert_rejected(h, line(HOME, [5, -90, 90, 0, 0, 0], 5), Result.INVALID_GOAL, '#AUT')


def test_aut_allowed_when_requested(bridge):
    h = bridge(sim_env={'OPMODE': 'AUT'}, allowed_modes='T1,AUT')
    target = [5, -90, 90, 0, 0, 0]
    _, result = h.send(line(HOME, target, 5))
    assert result.error_code == Result.SUCCESSFUL, result.error_string
    assert approx_axes(h.robot_axes(), target)


def test_wrong_program_rejected(bridge):
    h = bridge(sim_env={'PROG': 'ROS_SERVER'}, protocol='stream')
    assert_rejected(h, line(HOME, [5, -90, 90, 0, 0, 0], 5), Result.INVALID_GOAL, 'ROS_STREAM')


# --- cancelacion ---------------------------------------------------------------

def test_cancel_retracts_unread_points(bridge):
    h = bridge(sim_env={'STEP_S': '0.5'})
    pts, a = [], 0.0
    for c in range(8):                      # zigzag: 8 waypoints
        target = 4.0 if c % 2 == 0 else -4.0
        pts += line([a, -90, 90, 0, 0, 0], [target, -90, 90, 0, 0, 0], 8)
        a = target
    status, result = h.send(pts, cancel_after=0.8)
    assert 'cancelado' in result.error_string
    time.sleep(2.0)                         # el robot termina lo ya planificado
    wr, rd = int(h.read('COM_WR')), int(h.read('COM_RD'))
    assert wr == rd < 8, f'COM_WR={wr} COM_RD={rd}'
    executed = h.sim.output().count('seq ')
    assert executed == rd < 8
