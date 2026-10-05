"""Puente ROS 2 <-> KUKA KR C4 via KUKAVARPROXY.

- Publica /joint_states leyendo $AXIS_ACT (grados -> radianes).
- Action server control_msgs/FollowJointTrajectory. La trayectoria se simplifica
  en espacio articular (un PTP es una recta en ese espacio) y se envia segun
  `protocol`:
  - single (ros_server.src): un PTP por punto con COM_E6AXIS + COM_ACTION=1;
    el robot para en cada punto.
  - stream (ros_stream.src): cola circular COM_BUF[16] + COM_WR/COM_RD; el
    robot encadena los puntos con C_PTP y solo para en el ultimo.

Seguridad:
- allow_motion=false (por defecto): los goals se validan y se registran, pero
  NO se escribe nada en el robot.
- Cada segmento (incluido el primero, desde la posicion actual) se limita a
  max_step_deg por eje. Si max_total_deg > 0, ningun punto puede alejarse mas
  de eso de la posicion inicial (0 = sin limite, solo limites articulares).
- Antes de cada PTP: $MODE_OP debe estar en allowed_modes (por defecto solo T1)
  y el programa seleccionado debe ser ROS_SERVER.
- Tras COM_ACTION=0 se espera a que $AXIS_ACT llegue al destino; si el robot
  queda quieto lejos del destino, se aborta.
- Se valida contra los limites del URDF y los finales de carrera software del
  controlador ($SOFTN_END / $SOFTP_END), con un margen.
- Un PTP ya iniciado no se puede abortar desde aqui: cancelar solo evita que se
  envien los puntos siguientes (en stream, ademas, se retiran de la cola los que
  el robot aun no ha leido; los ya planificados, hasta $ADVANCE, se ejecutan).
  Para parar: soltar el pulsador de habilitacion.
"""

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from kuka_bridge.kvp_client import KvpClient, KvpError, format_e6axis, parse_axes

Result = FollowJointTrajectory.Result

BUF_SIZE = 16   # COM_BUF[16] en $config.dat

# Limites articulares del URDF kr10_r1100_2 (grados)
URDF_LOWER = [-170.0, -190.0, -120.0, -185.0, -120.0, -350.0]
URDF_UPPER = [170.0, 45.0, 156.0, 185.0, 120.0, 350.0]


def simplify_path(points, tol):
    """Ramer-Douglas-Peucker en espacio articular (grados).

    Conserva primer y ultimo punto. Un punto intermedio se elimina si esta a
    menos de tol (max por eje) de donde estaria sobre la recta entre los puntos
    conservados, recorrida en proporcion a la distancia acumulada del camino
    original. Asi una ida y vuelta sobre la misma recta no se fusiona.
    """
    cum = [0.0]
    for p, q in zip(points, points[1:]):
        cum.append(cum[-1] + math.dist(p, q))

    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 - i0 < 2:
            continue
        a, b = points[i0], points[i1]
        length = cum[i1] - cum[i0]
        worst, worst_i = -1.0, None
        for i in range(i0 + 1, i1):
            t = (cum[i] - cum[i0]) / length if length > 0 else 0.0
            dev = max(abs(pi - (ai + t * (bi - ai))) for pi, ai, bi in zip(points[i], a, b))
            if dev > worst:
                worst, worst_i = dev, i
        if worst > tol:
            keep.add(worst_i)
            stack += [(i0, worst_i), (worst_i, i1)]
    return [points[i] for i in sorted(keep)]


class KukaBridge(Node):
    def __init__(self):
        super().__init__('kuka_bridge')
        p = self.declare_parameter
        self.ip = p('robot_ip', '172.31.1.147').value
        self.port = p('robot_port', 7000).value
        self.joint_names = list(p('joint_names', [f'joint_{i}' for i in range(1, 7)]).value)
        rate = p('publish_rate', 20.0).value
        self.allow_motion = p('allow_motion', False).value
        self.max_step = p('max_step_deg', 10.0).value
        # 0 = sin limite de movimiento total (solo limites articulares)
        self.max_total = p('max_total_deg', 0.0).value
        self.path_tol = p('path_tolerance_deg', 0.5).value
        self.protocol = p('protocol', 'single').value
        if self.protocol not in ('single', 'stream'):
            raise ValueError(f'protocol debe ser single o stream, no {self.protocol!r}')
        self.stream_window = p('stream_window', 4).value
        self.limit_margin = p('limit_margin_deg', 1.0).value
        self.motion_timeout = p('motion_timeout_s', 120.0).value
        self.goal_tol = p('goal_tolerance_deg', 0.5).value
        self.settle_time = p('settle_time_s', 1.0).value
        # Separados por comas, p. ej. 'T1' o 'T1,AUT'
        self.allowed_modes = [m.strip().lstrip('#').upper()
                              for m in p('allowed_modes', 'T1').value.split(',') if m.strip()]
        self.expected_program = (p('expected_program', '').value or
                                 {'single': 'ROS_SERVER', 'stream': 'ROS_STREAM'}[self.protocol])
        self.poll_period = p('poll_period_s', 0.05).value
        action_name = p('action_name',
                        'joint_trajectory_controller/follow_joint_trajectory').value

        self.client = KvpClient(self.ip, self.port)
        self.lock = threading.Lock()   # un solo cliente para timer y action
        self.busy = False
        self.last_deg = None

        self.lower, self.upper = self._load_limits()

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self._publish_joint_states,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.action_server = ActionServer(
            self, FollowJointTrajectory, action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'KUKAVARPROXY {self.ip}:{self.port} | action "{action_name}" | '
            f'protocol={self.protocol} ({self.expected_program}) | '
            f'allow_motion={self.allow_motion} | max_step={self.max_step} deg | '
            f'max_total={self.max_total if self.max_total > 0 else "sin limite"} deg')
        if not self.allow_motion:
            self.get_logger().warn('allow_motion=false: los goals NO moveran el robot')
        if self.allow_motion and set(self.allowed_modes) - {'T1', 'T2'}:
            self.get_logger().warn(
                f'allowed_modes={self.allowed_modes}: en AUT/AUT_EXT el robot se movera '
                'SIN pulsador de habilitacion en cuanto llegue un goal')
        try:
            msg = self._check_controller()
        except KvpError as e:
            msg = f'no se pudo leer el estado del controlador: {e}'
        if msg:
            self.get_logger().warn(f'Controlador no listo para mover: {msg}')

    # --- comunicacion ---------------------------------------------------
    def _read(self, var):
        with self.lock:
            return self.client.read(var)

    def _write(self, var, value):
        with self.lock:
            return self.client.write(var, value)

    def _read_axes(self):
        deg = parse_axes(self._read('$AXIS_ACT'))
        self.last_deg = deg
        return deg

    def _load_limits(self):
        lower, upper = list(URDF_LOWER), list(URDF_UPPER)
        try:
            for i in range(6):
                lower[i] = max(lower[i], float(self._read(f'$SOFTN_END[{i + 1}]')))
                upper[i] = min(upper[i], float(self._read(f'$SOFTP_END[{i + 1}]')))
            src = 'URDF + $SOFTN_END/$SOFTP_END'
        except (KvpError, ValueError) as e:
            lower, upper = list(URDF_LOWER), list(URDF_UPPER)
            src = f'solo URDF (no se pudieron leer los limites software: {e})'
        lower = [v + self.limit_margin for v in lower]
        upper = [v - self.limit_margin for v in upper]
        self.get_logger().info(
            f'Limites ({src}, margen {self.limit_margin} deg): ' +
            ', '.join(f'A{i + 1} [{lo:.1f}, {hi:.1f}]'
                      for i, (lo, hi) in enumerate(zip(lower, upper))))
        return lower, upper

    # --- /joint_states ---------------------------------------------------
    def _publish_joint_states(self):
        try:
            deg = self._read_axes()
        except KvpError as e:
            self.get_logger().error(f'Lectura de $AXIS_ACT fallida: {e}',
                                    throttle_duration_sec=5.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [math.radians(v) for v in deg]
        self.js_pub.publish(msg)

    # --- FollowJointTrajectory --------------------------------------------
    def _on_goal(self, _goal):
        if self.busy:
            self.get_logger().warn('Goal rechazado: ya hay una trayectoria en curso')
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run_trajectory(goal_handle)
        except KvpError as e:
            self.get_logger().error(f'Error de comunicacion durante la trayectoria: {e}')
            goal_handle.abort()
            return self._result(Result.PATH_TOLERANCE_VIOLATED, f'comunicacion: {e}')
        finally:
            self.busy = False

    def _run_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory

        # 1. Reordenar a A1..A6 y pasar a grados
        if sorted(traj.joint_names) != sorted(self.joint_names):
            return self._fail(goal_handle, Result.INVALID_JOINTS,
                              f'joints {list(traj.joint_names)} != {self.joint_names}')
        idx = [list(traj.joint_names).index(n) for n in self.joint_names]
        points = []
        for k, pt in enumerate(traj.points):
            if len(pt.positions) != len(traj.joint_names):
                return self._fail(goal_handle, Result.INVALID_GOAL,
                                  f'punto {k} sin 6 posiciones')
            points.append([math.degrees(pt.positions[i]) for i in idx])
        if not points:
            return self._fail(goal_handle, Result.INVALID_GOAL, 'trayectoria vacia')

        # 2. Validar todos los puntos de MoveIt antes de mover nada
        current = self._read_axes()
        prev = current
        for k, pt in enumerate(points):
            for i, v in enumerate(pt):
                if not self.lower[i] <= v <= self.upper[i]:
                    return self._fail(
                        goal_handle, Result.INVALID_GOAL,
                        f'punto {k}: A{i + 1}={v:.2f} fuera de '
                        f'[{self.lower[i]:.1f}, {self.upper[i]:.1f}]')
            total = max(abs(a - b) for a, b in zip(pt, current))
            if self.max_total > 0 and total > self.max_total:
                return self._fail(
                    goal_handle, Result.INVALID_GOAL,
                    f'punto {k}: se aleja {total:.2f} deg de la posicion actual '
                    f'> max_total_deg={self.max_total}')
            step = max(abs(a - b) for a, b in zip(pt, prev))
            if step > self.max_step:
                return self._fail(
                    goal_handle, Result.INVALID_GOAL,
                    f'punto {k}: salto de {step:.2f} deg > max_step_deg={self.max_step}')
            prev = pt

        # 3. Simplificar: un PTP es una recta en espacio articular, asi que los
        #    puntos alineados (a menos de path_tolerance_deg) sobran
        waypoints = simplify_path([current] + points, self.path_tol)[1:]

        shown = list(enumerate(waypoints))
        if len(shown) > 6:
            shown = shown[:3] + [(None, None)] + shown[-2:]
        plan = '\n'.join('  ...' if k is None else f'  {k}: {format_e6axis(wp)}'
                         for k, wp in shown)
        self.get_logger().info(
            f'Trayectoria valida: {len(points)} puntos -> {len(waypoints)} waypoint(s) '
            f'[{self.protocol}]:\n{plan}')

        if not self.allow_motion:
            return self._fail(goal_handle, Result.INVALID_GOAL,
                              'allow_motion=false: no se envio nada al robot')

        not_ready = self._check_controller()
        if not_ready:
            return self._fail(goal_handle, Result.INVALID_GOAL, f'no enviado: {not_ready}')

        if self.protocol == 'stream':
            return self._exec_stream(goal_handle, waypoints)
        return self._exec_single(goal_handle, waypoints)

    # --- protocolo single (ros_server.src) --------------------------------
    def _exec_single(self, goal_handle, waypoints):
        if self._read('COM_ACTION').strip() != b'0':
            return self._fail(goal_handle, Result.INVALID_GOAL,
                              'COM_ACTION != 0: ros_server no esta listo')

        for k, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(Result.SUCCESSFUL, f'cancelado antes del waypoint {k}')

            not_ready = self._check_controller()
            if not_ready:
                return self._fail(goal_handle, Result.INVALID_GOAL,
                                  f'waypoint {k} no enviado: {not_ready}')

            self.get_logger().info(f'PTP {k + 1}/{len(waypoints)}')
            self._write('COM_E6AXIS', format_e6axis(wp))
            self._write('COM_ACTION', '1')

            t0 = time.monotonic()
            while self._read('COM_ACTION').strip() != b'0':
                if time.monotonic() - t0 > self.motion_timeout:
                    # Si el PTP no habia empezado, esto evita que arranque despues
                    self._write('COM_ACTION', '0')
                    return self._fail(
                        goal_handle, Result.PATH_TOLERANCE_VIOLATED,
                        f'timeout ({self.motion_timeout} s) en waypoint {k}. '
                        'COM_ACTION puesto a 0; si el PTP ya estaba en marcha, terminara')
                self._feedback(goal_handle, wp)
                time.sleep(self.poll_period)

            # COM_ACTION=0 no basta: esperar a que $AXIS_ACT llegue al destino
            err = self._wait_arrival(goal_handle, wp)
            if err > self.goal_tol:
                return self._fail(
                    goal_handle, Result.GOAL_TOLERANCE_VIOLATED,
                    f'waypoint {k}: COM_ACTION volvio a 0 pero el robot esta parado '
                    f'a {err:.3f} deg del destino')

        goal_handle.succeed()
        self.get_logger().info(f'Trayectoria completada (error {err:.3f} deg)')
        return self._result(Result.SUCCESSFUL, '')

    # --- protocolo stream (ros_stream.src) --------------------------------
    def _read_int(self, var):
        return int(self._read(var).strip())

    def _stream_retract(self):
        """Retira de la cola los puntos que el robot aun no ha leido."""
        rd = self._read_int('COM_RD')
        self._write('COM_WR', str(rd))
        # El robot pudo leer uno justo antes de la escritura: ese se ejecuta
        time.sleep(0.1)
        rd2 = self._read_int('COM_RD')
        if rd2 > rd:
            self._write('COM_WR', str(rd2))
        return max(rd, rd2)

    def _exec_stream(self, goal_handle, waypoints):
        rd, wr = self._read_int('COM_RD'), self._read_int('COM_WR')
        if rd < wr:
            return self._fail(goal_handle, Result.INVALID_GOAL,
                              f'cola no vacia (COM_RD={rd} < COM_WR={wr}): '
                              'ros_stream tiene puntos pendientes')
        base = max(rd, wr)
        if wr < base:
            self._write('COM_WR', str(base))
        gen = self._read_int('COM_GEN')
        last = base + len(waypoints)
        window = min(max(int(self.stream_window), 1), BUF_SIZE - 1)
        final = waypoints[-1]

        self._write('COM_LAST', str(last))
        self.get_logger().info(f'Stream: secuencias {base + 1}..{last}')

        sent, last_rd = base, rd
        last_axes = self._read_axes()
        progress_t = time.monotonic()
        while True:
            rd = self._read_int('COM_RD')

            if goal_handle.is_cancel_requested:
                rd = self._stream_retract()
                goal_handle.canceled()
                msg = (f'cancelado: el robot termina los puntos ya planificados '
                       f'(hasta secuencia {rd})')
                self.get_logger().warn(msg)
                return self._result(Result.SUCCESSFUL, msg)

            if self._read_int('COM_GEN') != gen:
                return self._fail(goal_handle, Result.PATH_TOLERANCE_VIOLATED,
                                  'ros_stream se ha reiniciado durante la trayectoria')
            not_ready = self._check_controller()
            if not_ready:
                self._stream_retract()
                return self._fail(goal_handle, Result.PATH_TOLERANCE_VIOLATED,
                                  f'controlador no listo: {not_ready}')

            # Mantener hasta `window` puntos sin leer por delante del robot
            while sent < last and sent - rd < window:
                seq = sent + 1
                slot = (seq - 1) % BUF_SIZE + 1
                self._write(f'COM_BUF[{slot}]', format_e6axis(waypoints[seq - base - 1]))
                self._write('COM_WR', str(seq))
                sent = seq

            axes = self._read_axes()
            now = time.monotonic()
            moving = max(abs(a - b) for a, b in zip(axes, last_axes)) > 0.01
            if rd != last_rd or moving:
                progress_t = now
            last_rd, last_axes = rd, axes

            # El ultimo PTP es exacto: esperar a que el robot este parado en destino,
            # no solo dentro de la tolerancia mientras aun entra
            err = max(abs(a - b) for a, b in zip(axes, final))
            if rd >= last and err <= self.goal_tol and not moving:
                goal_handle.succeed()
                self.get_logger().info(f'Trayectoria completada (error {err:.3f} deg)')
                return self._result(Result.SUCCESSFUL, '')

            if now - progress_t > self.motion_timeout:
                self._stream_retract()
                return self._fail(goal_handle, Result.PATH_TOLERANCE_VIOLATED,
                                  f'sin progreso en {self.motion_timeout} s '
                                  f'(COM_RD={rd}, ultimo={last}, error {err:.2f} deg)')

            self._feedback(goal_handle, final)
            time.sleep(self.poll_period)

    def _check_controller(self):
        """Devuelve un mensaje si el controlador no esta listo para mover, o None."""
        mode = self._read('$MODE_OP').decode().strip().lstrip('#')
        if mode not in self.allowed_modes:
            return f'$MODE_OP=#{mode} no esta en allowed_modes={self.allowed_modes}'
        prog = self._read('$PRO_NAME1[]').decode().strip().strip('"').strip()
        if prog.upper() != self.expected_program.upper():
            return (f'programa seleccionado "{prog}" != "{self.expected_program}" '
                    f'(seleccionar y arrancar {self.expected_program.lower()}.src)')
        return None

    def _wait_arrival(self, goal_handle, target):
        """Espera a que el robot llegue a target o se quede quieto settle_time_s.

        Devuelve el error final (grados, max por eje).
        """
        t0 = time.monotonic()
        prev = self._read_axes()
        still_since = time.monotonic()
        while True:
            err = max(abs(a - b) for a, b in zip(prev, target))
            if err <= self.goal_tol:
                return err
            now = time.monotonic()
            if now - still_since > self.settle_time or now - t0 > self.motion_timeout:
                return err
            self._feedback(goal_handle, target)
            time.sleep(self.poll_period)
            cur = self._read_axes()
            if max(abs(a - b) for a, b in zip(cur, prev)) > 0.01:
                still_since = time.monotonic()
            prev = cur

    def _feedback(self, goal_handle, target_deg):
        if self.last_deg is None:
            return
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.joint_names = self.joint_names
        fb.desired = JointTrajectoryPoint(positions=[math.radians(v) for v in target_deg])
        fb.actual = JointTrajectoryPoint(positions=[math.radians(v) for v in self.last_deg])
        fb.error = JointTrajectoryPoint(
            positions=[a - b for a, b in zip(fb.desired.positions, fb.actual.positions)])
        goal_handle.publish_feedback(fb)

    def _fail(self, goal_handle, code, msg):
        self.get_logger().error(f'Goal abortado: {msg}')
        goal_handle.abort()
        return self._result(code, msg)

    @staticmethod
    def _result(code, msg):
        r = Result()
        r.error_code = code
        r.error_string = msg
        return r


def main():
    rclpy.init()
    node = KukaBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.client.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
