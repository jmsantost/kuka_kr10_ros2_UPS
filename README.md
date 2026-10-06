# kuka_kr10_ros2_UPS

Control de un **KUKA KR10 R1100-2 (KR AGILUS)** con **ROS 2 Jazzy y MoveIt 2**
desde un controlador **KR C4 sin licencias RSI ni EKI**, usando
[KUKAVARPROXY](https://github.com/ImtsSrl/KUKAVARPROXY) para leer y escribir
variables KRL por TCP.

```
┌────────────── PC (Ubuntu 24.04, ROS 2 Jazzy) ──────────────┐        ┌────── KR C4 (KSS 8.6) ──────┐
│                                                             │        │                             │
│  RViz ── MoveIt 2 (move_group) ── escena de colision        │        │  KUKAVARPROXY (TCP 7000)    │
│                 │ FollowJointTrajectory                     │  TCP   │        │                    │
│                 ▼                                           │ ◀────▶ │  variables globales KRL     │
│           kuka_bridge  ── /joint_states (20 Hz) ──▶ RViz    │  KLI   │        │                    │
│                                                             │        │  ros_stream.src / ros_server│
└─────────────────────────────────────────────────────────────┘        └─────────────────────────────┘
```

- `/joint_states` a 20 Hz leyendo `$AXIS_ACT` (grados → radianes).
- Action server `control_msgs/FollowJointTrajectory`
  (`/joint_trajectory_controller/follow_joint_trajectory`), que es lo que MoveIt espera.
- Dos protocolos de ejecución:
  - **stream** (recomendado): cola circular de 16 puntos; el robot los encadena con
    `C_PTP` sin parar y solo frena en el último.
  - **single**: un PTP por punto; el robot para en cada uno. Más simple, útil como respaldo.
- Escena de colisión configurable (suelo, pedestal, paredes del espacio de trabajo, mesa).
- Varias capas de seguridad (ver [Seguridad](#seguridad)).

> **No es tiempo real.** KUKAVARPROXY es un proxy de variables: sirve para mover el
> robot punto a punto con trayectorias planificadas, no para control en lazo cerrado
> ni servoing. Para eso hace falta RSI.

La historia completa del desarrollo, las decisiones y los problemas encontrados están en
[docs/desarrollo.md](docs/desarrollo.md).

---

## Contenido del repositorio

| Ruta | Qué es |
|---|---|
| `kuka_bridge/kuka_bridge_node.py` | Nodo puente: `/joint_states`, action server, validación y protocolos |
| `kuka_bridge/kvp_client.py` | Cliente KUKAVARPROXY propio (timeouts, reconexión, lectura completa de mensajes) |
| `kuka_bridge/scene_publisher.py` | Carga `config/scene.yaml` en MoveIt |
| `kuka_bridge/read_vars.py` | `kuka_vars`: lee variables KRL (solo lectura) |
| `kuka_bridge/go_home.py` | `go_home`: lleva el robot a `home` (u otro estado del SRDF) con MoveIt |
| `krl/` | Programas KRL y declaraciones para `$config.dat` |
| `config/scene.yaml` | Escena de colisión |
| `config/moveit_controllers.yaml` | Conexión MoveIt → bridge |
| `config/home.yaml` | Posición home para el visor de URDF |
| `launch/` | `moveit.launch.py`, `bridge.launch.py`, `view_robot.launch.py` |
| `tools/kvp_sim.py` | Simulador de KUKAVARPROXY + KRL para probar sin robot |

El paquete ROS se llama `kuka_bridge`.

---

## Requisitos

**Robot**
- KR C4 con KSS 8.x (probado en 8.6.6).
- KUKAVARPROXY instalado en el controlador y arrancando con Windows (puerto TCP 7000).
- Red: el PC conectado a la interfaz **KLI** del controlador en la misma subred
  (en nuestro caso robot `172.31.1.147/16`, PC `172.31.1.100/16`).

**PC**
- Ubuntu 24.04 + ROS 2 Jazzy.
- Paquetes:
  ```bash
  sudo apt install ros-jazzy-moveit ros-jazzy-control-msgs \
                   ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
  ```

---

## Instalación

```bash
mkdir -p ~/kuka_ws/src && cd ~/kuka_ws/src
git clone https://github.com/jmsantost/kuka_kr10_ros2_UPS.git
git clone --depth 1 https://github.com/kroshu/kuka_robot_descriptions.git   # URDF + MoveIt de KUKA

cd ~/kuka_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select kuka_resources kuka_agilus_support kuka_kr_moveit_config kuka_bridge
source install/setup.bash
```

Solo se compilan los paquetes de `kuka_robot_descriptions` necesarios para el KR10 R1100-2.

---

## Configuración del robot (smartPAD, como Experto)

### 1. Variables globales

Añadir en `KRC:\R1\System\$config.dat`, sección **USER GLOBALS → Userdefined Variables**
(fichero [krl/config_dat_globals.txt](krl/config_dat_globals.txt)):

```
DECL E6AXIS COM_E6AXIS
DECL INT COM_ACTION=0

DECL E6AXIS COM_BUF[16]
DECL INT COM_WR=0
DECL INT COM_RD=0
DECL INT COM_LAST=0
DECL INT COM_GEN=0
```

### 2. Programas

Crear en `KRC:\R1\Program` uno o los dos programas:

- [krl/ros_stream.src](krl/ros_stream.src) — protocolo `stream` (movimiento fluido).
- [krl/ros_server.src](krl/ros_server.src) — protocolo `single`.

Velocidad: la fija `BAS(#VEL_PTP, 10)` en el programa multiplicada por el override
(`$OV_PRO`), **no** el escalado de velocidad de MoveIt.

### Notas del editor KRL
- El teclado del controlador suele estar en **alemán**: `[` = AltGr+8, `]` = AltGr+9
  (sin AltGr: Ctrl+Alt+8/9). También se pueden copiar de otra línea de `$config.dat`.
- **KRL no tiene operador `MOD`**: el resto se calcula con división entera de `INT`
  (`a - (a / 16) * 16`).
- En `ros_stream.src` **no usar `WAIT FOR`** dentro del bucle: detiene el *advance run*
  y se pierde la aproximación entre movimientos.

### 3. Antes de mover
Seleccionar el programa, arrancarlo (BCO: `PTP $AXIS_ACT`, no se mueve) y comprobar:

```bash
ros2 run kuka_bridge kuka_vars
```
```
$MODE_OP       #T1
$PRO_NAME1[]   "ROS_STREAM"
COM_WR         0
COM_RD         0
...
```

---

## Uso

### MoveIt + robot real
```bash
source ~/kuka_ws/install/setup.bash
ros2 launch kuka_bridge moveit.launch.py allow_motion:=true protocol:=stream
```

En RViz, panel **MotionPlanning**:
1. *Planning* → **Start State** `<current>`, **Goal State** `home` / `<current>`.
2. Para mover ejes concretos: pestaña **Joints**. (Arrastrar la esfera cambia también la
   orientación y suele mover varios ejes.)
3. **Plan** → revisar la animación → **Execute**.

En T1 hay que mantener el pulsador de habilitación y Start durante el movimiento.

### Ir a home
Con `moveit.launch.py` en marcha, en otra terminal:
```bash
ros2 run kuka_bridge go_home              # planifica, muestra el cambio por eje y pide confirmación
ros2 run kuka_bridge go_home -y           # sin confirmación
ros2 run kuka_bridge go_home --plan-only  # solo planificar, no mueve
ros2 run kuka_bridge go_home --state X    # otro group_state del SRDF
```
Lee el estado del SRDF que usa MoveIt, planifica evitando la escena y ejecuta a través del
bridge (con todas sus validaciones). Si el robot ya está en el destino, no hace nada.

### Solo bridge (sin MoveIt)
```bash
ros2 launch kuka_bridge bridge.launch.py allow_motion:=true protocol:=stream
```
Ejemplo: cuadrado pequeño de 5° en A1/A2 desde home:
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [
{positions: [0.0872665, -1.570796, 1.570796, 0, 0, 0], time_from_start: {sec: 1}},
{positions: [0.0872665, -1.483530, 1.570796, 0, 0, 0], time_from_start: {sec: 2}},
{positions: [0.0,       -1.483530, 1.570796, 0, 0, 0], time_from_start: {sec: 3}},
{positions: [0.0,       -1.570796, 1.570796, 0, 0, 0], time_from_start: {sec: 4}}]}}"
```

### Visor del URDF (sin robot)
```bash
ros2 launch kuka_bridge view_robot.launch.py
```

### Argumentos de los launch

| Argumento | Defecto | |
|---|---|---|
| `robot_ip` / `robot_port` | `172.31.1.147` / `7000` | KUKAVARPROXY |
| `allow_motion` | `false` | sin `true` nunca escribe movimiento |
| `protocol` | `single` | `single` (ros_server) o `stream` (ros_stream) |
| `allowed_modes` | `T1` | modos permitidos, p. ej. `T1,AUT` |
| `max_step_deg` | `10.0` | salto máximo entre puntos consecutivos de la trayectoria |
| `max_total_deg` | `0.0` | movimiento total máximo por eje (0 = sin límite) |
| `rviz` | `true` | abrir RViz |
| `scene` | `true` | cargar `config/scene.yaml` (solo `moveit.launch.py`) |

---

## Convención de ejes

`joint_i = radianes(Ai)`, mismo signo, verificado en el robot real. Home:
A1 0°, A2 −90°, A3 90°, A4 0°, A5 0°, A6 0° (estado `home` del SRDF).

Frame de la escena (`base_link` = ROBROOT): origen en el eje A1 a la altura de la
superficie de apoyo del robot; **+X** hacia donde apunta el brazo con A1 = 0, **+Y** a la
izquierda, **+Z** arriba. A1 positivo gira en sentido horario visto desde arriba.

---

## Seguridad

El bridge solo escribe movimiento si **todo** esto se cumple:

1. `allow_motion:=true`. Con `false` valida y muestra la trayectoria, pero no envía nada.
2. `$MODE_OP` está en `allowed_modes` (por defecto solo **T1**). En AUT el robot se mueve
   **sin pulsador de habilitación** en cuanto llega un goal.
3. El programa seleccionado es el del protocolo (`ROS_STREAM` / `ROS_SERVER`).
4. Todos los puntos están dentro de los límites articulares (URDF ∩ `$SOFTN_END`/`$SOFTP_END`,
   con 1° de margen).
5. Ningún salto entre puntos consecutivos supera `max_step_deg`.
6. Si `max_total_deg > 0`, ningún punto se aleja más de eso de la posición inicial.

Además:
- Se valida la trayectoria **completa antes de mover nada**.
- La trayectoria solo se da por terminada cuando `$AXIS_ACT` está en destino **y parado**.
- Se detecta si el programa KRL se reinicia en mitad de una trayectoria (`COM_GEN`).
- **Cancelar no frena un movimiento en marcha**: en `stream` se retiran de la cola los
  puntos no leídos y el robot termina los ya planificados (hasta `$ADVANCE` = 3).
  **Para parar: soltar el pulsador, Stop o seta de emergencia.**
- La escena de colisión solo protege de lo que está modelado. El gripper aún no lo está.

---

## Escena de colisión

[config/scene.yaml](config/scene.yaml): cajas con `size`, `position` (centro), `yaw` y
`color`, en el frame `base_link`. Configuración actual:

| Objeto | Medidas | Posición |
|---|---|---|
| Pedestal | 36 × 36 × 40 cm | centrado bajo el robot |
| Suelo | — | z = −0.40 m |
| Paredes | — | 0.90 m detrás, 1.10 m delante y a los lados |
| Mesa | 46 (X) × 80.5 (Y) × 62 (alto) cm | delante, centrada, borde a 0.50 m del eje A1 |

Tras editarla: `colcon build --packages-select kuka_bridge` y relanzar.

---

## Protocolos

### single (`ros_server.src`)
Por cada punto: escribir `COM_E6AXIS` → `COM_ACTION=1` → el robot hace `PTP` →
`WAIT SEC 0` (para el advance run) → `COM_ACTION=0`. El bridge espera además a que
`$AXIS_ACT` llegue al destino.

### stream (`ros_stream.src`)
- El PC escribe el punto de secuencia `s` en `COM_BUF[(s-1) mod 16 + 1]` y luego `COM_WR = s`.
- El robot, en un `LOOP` con `IF COM_RD < COM_WR` (las variables globales de usuario no
  paran el advance run), planifica el punto con `PTP ... C_PTP` y suma `COM_RD`.
- `COM_LAST` marca el último punto, que va sin `C_PTP` para parar exacto.
- El bridge mantiene como máximo 4 puntos sin leer por delante (`stream_window`).
- `COM_GEN` se incrementa al arrancar el programa: si cambia durante una trayectoria,
  el bridge aborta.

### Simplificación de la trayectoria
MoveIt envía decenas de puntos. Un PTP de KUKA es una recta en espacio articular, así que
el bridge elimina los puntos alineados (Ramer-Douglas-Peucker en espacio articular,
parametrizado por longitud de arco, tolerancia `path_tolerance_deg` = 0.5°). Un giro de un
solo eje queda en 1 PTP.

---

## Probar sin robot

[tools/kvp_sim.py](tools/kvp_sim.py) imita KUKAVARPROXY y el programa KRL. **Usar siempre
un dominio ROS aislado**, para que ningún goal de prueba llegue a un bridge conectado al
robot real:

```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
PROG=ROS_STREAM python3 ~/kuka_ws/src/kuka_kr10_ros2_UPS/tools/kvp_sim.py &
ros2 launch kuka_bridge moveit.launch.py robot_ip:=127.0.0.1 robot_port:=17000 \
    allow_motion:=true protocol:=stream
```

---

## Pendiente

- Modelo del gripper en la escena (adjunto a `tool0`).
- Control del gripper desde ROS (salidas `$OUT[n]` por KUKAVARPROXY).
- Movimientos cartesianos `LIN` (hoy todo se envía como PTP articulares).
- Pasar el escalado de velocidad de MoveIt al robot.

## Créditos

- [kroshu/kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions): URDF,
  mallas y configuración de MoveIt.
- [ImtsSrl/KUKAVARPROXY](https://github.com/ImtsSrl/KUKAVARPROXY) y
  [py-openshowvar](https://github.com/linuxsand/py_openshowvar) (protocolo).

Licencia: Apache-2.0.
