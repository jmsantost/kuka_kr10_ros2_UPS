"""Lleva el robot a un estado con nombre del SRDF (por defecto `home`) usando MoveIt.

    ros2 run kuka_bridge go_home                 # planifica, muestra y pide confirmacion
    ros2 run kuka_bridge go_home -y              # sin confirmacion
    ros2 run kuka_bridge go_home --plan-only     # solo planificar (no mueve)
    ros2 run kuka_bridge go_home --state otro    # otro group_state del SRDF

Requiere moveit.launch.py en marcha. La trayectoria evita la escena de colision y
se ejecuta a traves de kuka_bridge, con todas sus validaciones.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUP = 'manipulator'
TOLERANCE = math.radians(0.01)   # por debajo del umbral de "ya esta" (0.05 deg)


def error_name(code):
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == code:
            return name
    return str(code)


class GoHome(Node):
    def __init__(self):
        super().__init__('go_home')
        self.joints = None
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)

    def _on_js(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('tiempo de espera agotado')
        return future.result()

    def named_state(self, state):
        """Lee <group_state name=state> del SRDF publicado por move_group."""
        client = self.create_client(GetParameters, '/move_group/get_parameters')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('move_group no responde: lanza moveit.launch.py')
        srdf = self.wait(client.call_async(
            GetParameters.Request(names=['robot_description_semantic'])), 10.0)
        root = ET.fromstring(srdf.values[0].string_value)
        for gs in root.iter('group_state'):
            if gs.get('name') == state and gs.get('group') == GROUP:
                return {j.get('name'): float(j.get('value')) for j in gs.iter('joint')}
        names = sorted({gs.get('name') for gs in root.iter('group_state')})
        raise RuntimeError(f'el SRDF no tiene el estado "{state}" (hay: {names})')

    def current(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.joints is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joints is None:
            raise RuntimeError('no llega /joint_states: esta kuka_bridge en marcha?')
        return self.joints

    def plan(self, target):
        client = ActionClient(self, MoveGroup, 'move_action')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('no esta el action server move_action')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=TOLERANCE,
                            tolerance_below=TOLERANCE, weight=1.0)
            for n, v in target.items()])]
        goal.planning_options.plan_only = True
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('move_group rechazo la peticion')
        res = self.wait(handle.get_result_async(), 30.0).result
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'no se pudo planificar: {error_name(res.error_code.val)}')
        return res.planned_trajectory

    def execute(self, trajectory):
        client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('no esta el action server execute_trajectory')
        handle = self.wait(client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)), 10.0)
        if not handle.accepted:
            raise RuntimeError('move_group rechazo la ejecucion')
        # Sin limite: en T1 depende de que alguien mantenga el pulsador
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        code = future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'la ejecucion fallo: {error_name(code)} '
                               '(ver el log de kuka_bridge)')


def main():
    parser = argparse.ArgumentParser(description='Mover el robot a un estado del SRDF')
    parser.add_argument('--state', default='home', help='group_state del SRDF (home)')
    parser.add_argument('-y', '--yes', action='store_true', help='no pedir confirmacion')
    parser.add_argument('--plan-only', action='store_true', help='solo planificar')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = GoHome()
    try:
        target = node.named_state(args.state)
        now = node.current()
        print(f'Estado "{args.state}":')
        print('  eje     actual   destino   cambio (grados)')
        changes = []
        for i, (name, value) in enumerate(sorted(target.items()), start=1):
            cur = math.degrees(now.get(name, float('nan')))
            dst = math.degrees(value)
            changes.append(abs(dst - cur))
            print(f'  A{i}  {cur:9.2f} {dst:9.2f} {dst - cur:+9.2f}')
        if max(changes) < 0.05:
            print(f'Ya esta en "{args.state}", no hace falta moverse')
            return 0

        traj = node.plan(target)
        pts = traj.joint_trajectory.points
        dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 if pts else 0
        print(f'Plan: {len(pts)} puntos (duracion MoveIt {dur:.1f} s; la real la fija el robot)')

        if args.plan_only:
            print('--plan-only: no se ejecuta')
            return 0
        if not args.yes:
            answer = input('Ejecutar en el robot? [s/N] ').strip().lower()
            if answer not in ('s', 'si', 'sí', 'y', 'yes'):
                print('Cancelado, no se ha movido nada')
                return 0
        node.execute(traj)
        print(f'En "{args.state}"')
        return 0
    except (RuntimeError, KeyboardInterrupt, EOFError) as e:
        print(f'go_home: {e or "interrumpido"}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
