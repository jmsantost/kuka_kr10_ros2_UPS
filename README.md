# kuka_bridge

ROS 2 Jazzy <-> KUKA KR10 R1100-2 (KR C4, KSS 8.6) sin RSI ni EKI, a traves de
KUKAVARPROXY (TCP 7000). Publica `/joint_states` y ofrece la action
`/joint_trajectory_controller/follow_joint_trajectory` para MoveIt 2.

## Lado robot (`krl/`)
- `config_dat_globals.txt`: variables a declarar en `$config.dat`.
- `ros_server.src`: un PTP por punto (`protocol:=single`).
- `ros_stream.src`: cola de 16 puntos con `C_PTP`, movimiento fluido (`protocol:=stream`).

## Uso
```bash
source ~/kuka_ws/install/setup.bash
ros2 launch kuka_bridge moveit.launch.py allow_motion:=true protocol:=stream allowed_modes:=T1,AUT
ros2 launch kuka_bridge bridge.launch.py ...   # solo bridge + RViz, sin MoveIt
ros2 launch kuka_bridge view_robot.launch.py   # URDF con sliders, sin robot
ros2 run kuka_bridge kuka_vars                 # leer variables KRL (solo lectura)
```

## Seguridad (parametros del bridge)
| Parametro | Defecto | |
|---|---|---|
| `allow_motion` | `false` | sin esto nunca escribe movimiento |
| `allowed_modes` | `T1` | p. ej. `T1,AUT`; en AUT se mueve sin pulsador |
| `max_step_deg` | 10 | salto maximo entre puntos consecutivos |
| `max_total_deg` | 0 | movimiento total maximo (0 = sin limite) |
| limites articulares | URDF + `$SOFTN_END/$SOFTP_END` | margen 1 deg |

Ademas comprueba que el programa seleccionado sea el del protocolo y, al final,
que el robot este parado en destino.

## Escena de colision
`config/scene.yaml` (frame `base_link`: origen en el eje A1 sobre el pedestal de
0.40 m, +X hacia donde apunta el brazo en home). Suelo, pedestal, paredes del
espacio de trabajo y mesa. Falta el gripper.
