"""Publica la escena de colision (cajas de config/scene.yaml) en MoveIt.

Espera al servicio /apply_planning_scene de move_group, aplica los objetos y
termina. Volver a lanzarlo reemplaza los objetos con el mismo nombre.
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


def box(name, frame, size, position, yaw_deg):
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = frame
    obj.operation = CollisionObject.ADD
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    half = math.radians(yaw_deg) / 2.0
    pose.orientation.z, pose.orientation.w = math.sin(half), math.cos(half)
    obj.primitive_poses = [pose]
    return obj


def main():
    rclpy.init()
    node = Node('scene_publisher', automatically_declare_parameters_from_overrides=True)
    log = node.get_logger()
    frame = node.get_parameter('frame').value
    names = list(node.get_parameter('objects').value)

    scene = PlanningScene(is_diff=True)
    for name in names:
        size = list(node.get_parameter(f'{name}.size').value)
        pos = list(node.get_parameter(f'{name}.position').value)
        yaw = float(node.get_parameter(f'{name}.yaw').value)
        if len(size) != 3 or len(pos) != 3:
            raise ValueError(f'{name}: size y position deben tener 3 valores')
        scene.world.collision_objects.append(box(name, frame, size, pos, yaw))
        color = None
        if node.has_parameter(f'{name}.color'):
            color = [float(c) for c in node.get_parameter(f'{name}.color').value]
            if len(color) != 4:
                raise ValueError(f'{name}: color debe ser [r, g, b, alpha]')
            scene.object_colors.append(ObjectColor(id=name, color=ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=color[3])))
        log.info(f'{name}: size={size} position={pos} yaw={yaw} color={color}')
    scene.robot_state.is_diff = True

    client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    while not client.wait_for_service(timeout_sec=2.0):
        log.info('Esperando a move_group (/apply_planning_scene)...')
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future)
    ok = future.result() is not None and future.result().success
    (log.info if ok else log.error)(
        f'Escena {"aplicada" if ok else "NO aplicada"}: {len(names)} objeto(s)')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
