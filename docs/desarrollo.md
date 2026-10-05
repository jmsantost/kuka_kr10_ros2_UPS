# Desarrollo

Registro de cómo se construyó este proyecto, qué se decidió y por qué, y qué problemas
aparecieron. Útil para entender el código o para reproducirlo con otro robot KUKA.

## Punto de partida

- **Robot:** KUKA KR10 R1100-2 (KR AGILUS) sobre un pedestal de 40 cm.
- **Controlador:** KR C4, KSS 8.6.6, Windows 10 interno. Sin licencia RSI ni EKI, así que
  no se pueden usar los drivers oficiales de ROS 2 (`kuka_drivers` de kroshu).
- **Comunicación:** KUKAVARPROXY v9.0 (ImtsSrl) en el controlador, puerto TCP 7000,
  arrancando con Windows. Permite leer y escribir variables KRL. No es tiempo real.
- **Red:** interfaz KLI del robot `172.31.1.147/16`; portátil con Ubuntu 24.04 en
  `172.31.1.100/16` por cable (perfil de NetworkManager `kuka`), con el WiFi activo para
  internet.
- **Ya funcionaba:** un script en Python con `py-openshowvar` que leía `$AXIS_ACT`, y otro
  que movía A1 unos grados usando un programa KRL `ros_server.src` (`WAIT FOR COM_ACTION==1`
  → `PTP COM_E6AXIS` → `COM_ACTION=0`).

**Objetivo:** controlar el robot desde ROS 2 Jazzy con MoveIt 2.

---

## 1. Workspace y modelo del robot

- Workspace `~/kuka_ws`.
- URDF y mallas de [kroshu/kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions)
  (paquete `kuka_agilus_support`, modelo `kr10_r1100_2`). Solo se compilan los paquetes
  necesarios.
- Nombres de joints: `joint_1`…`joint_6`. Límites del URDF (grados): A1 ±170,
  A2 −190/+45, A3 −120/+156, A4 ±185, A5 ±120, A6 ±350.

**Verificación de la convención de ejes.** Con cinemática directa sobre el URDF:
- Con todos los ejes a 0 el brazo queda estirado en horizontal, que es el cero mecánico de KUKA.
- Con A2 −90° y A3 90° el brazo queda vertical y el antebrazo horizontal, igual que la
  posición home del robot real.
- A1 positivo gira en sentido horario visto desde arriba, que es la convención KUKA.

Conclusión: `joint_i = radianes(Ai)`, sin cambios de signo. Se confirmó después
moviendo el robot real.

Se añadió `view_robot.launch.py` con los sliders arrancando en home
(`config/home.yaml`, parámetro `zeros` de `joint_state_publisher`).

---

## 2. El bridge

### Cliente KUKAVARPROXY propio
`py-openshowvar` funciona, pero:
- No tiene timeouts: si el robot no responde, se bloquea.
- Hace un solo `recv(256)`, que puede cortar respuestas largas.
- No reconecta. KUKAVARPROXY cierra las conexiones inactivas tras 30 s.
- Estaba instalado en un venv que el Python de ROS no ve.

Por eso se escribió [kvp_client.py](../kuka_bridge/kvp_client.py), con el mismo protocolo:
cabecera `msg_id` + longitud, cuerpo `flag` (0 lectura / 1 escritura) + nombre + valor, y
respuesta con los 3 últimos bytes de estado (último `0x01` = OK). Añade lectura completa
según la cabecera, timeouts, reconexión con un reintento y errores como excepciones.

### Nodo
- **Timer a 20 Hz:** lee `$AXIS_ACT` y publica `/joint_states` en radianes.
- **Action server `FollowJointTrajectory`:** usa el nombre que espera la configuración de
  MoveIt de kroshu (`joint_trajectory_controller/follow_joint_trajectory`).
- **Un único cliente TCP compartido**, protegido con un lock, y `MultiThreadedExecutor`
  para que `/joint_states` se siga publicando mientras se ejecuta una trayectoria.

### Validación antes de mover
- Límites articulares: los del URDF combinados con los del controlador (`$SOFTN_END` y
  `$SOFTP_END`, leídos al arrancar), con 1° de margen.
- `max_step_deg`: salto máximo entre puntos consecutivos, contando el primero desde la
  posición actual.
- `max_total_deg`: distancia máxima a la posición inicial. Empezó en 15°, se subió a 30° y
  90°, y quedó desactivado (0) una vez hubo escena de colisión.
- `allow_motion=false` por defecto: valida y muestra lo que enviaría, pero no escribe nada.

---

## 3. Problemas encontrados con el robot real

### `COM_ACTION` volvía a 0 sin que el robot se hubiera movido
**Síntoma:** en el primer movimiento real, el bridge dio el goal por terminado con un
error de 5°, exactamente la distancia pedida.

**Causa:** el programa no estaba arrancado. Al arrancarlo después, la línea
`COM_ACTION = 0` del principio de `ros_server.src` borró la petición.

**Solución en el bridge:**
- Antes de mover, comprueba que el programa seleccionado (`$PRO_NAME1[]`) es el esperado.
- Tras `COM_ACTION=0`, espera a que `$AXIS_ACT` llegue de verdad al destino. Si el robot
  se queda quieto lejos del destino, aborta.

### El controlador estaba en AUT con override al 100 %
Al leer `$MODE_OP` se vio que el robot estaba en **automático**: un goal lo movía sin
nadie en el pulsador de habilitación. Se añadió `allowed_modes`, que por defecto solo
permite **T1**. Para usar AUT hay que pedirlo de forma explícita (`allowed_modes:=T1,AUT`),
y al arrancar el bridge avisa.

### Las pruebas con el simulador llegaron al robot real
Durante el desarrollo se probaba con un simulador local mientras el bridge real estaba
lanzado con `allow_motion:=true`. Ambos usaban el mismo nombre de action y el mismo dominio
ROS, así que los goals de prueba también los recibió el bridge real, que movió el robot unos
2° en A3 y A5.

**Lección:** las pruebas sin robot se hacen **siempre** en un dominio aislado
(`ROS_DOMAIN_ID` propio y `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), comprobando antes
con `ros2 action info` que solo existe el servidor de prueba. Por eso el
[simulador](../tools/kvp_sim.py) solo escucha en `127.0.0.1` y su documentación lo exige.

---

## 4. MoveIt 2

- Se usó `kuka_kr_moveit_config` de kroshu: SRDF con el grupo `manipulator` y el estado
  `home`, KDL y OMPL.
- [moveit_controllers.yaml](../config/moveit_controllers.yaml) propio: desactiva
  `execution_duration_monitoring`. Con PTP que paran en cada punto y en T1, que depende de
  que alguien mantenga el pulsador, la duración real no tiene nada que ver con la
  planificada, y MoveIt cancelaría la ejecución.

**Problema: RViz no cargaba el modelo y no aparecía la esfera interactiva.**
El panel MotionPlanning copia los parámetros de `move_group` y falla si en RViz ya existe
un parámetro numérico con el mismo nombre
(`parameter ... is of type {double}, setting it to {string} is not allowed`). Solución: a
RViz solo se le pasa el nombre del solver de cinemática, que es lo mismo que hace el launch
de kroshu.

También se quitó de la configuración de RViz el panel `RvizVisualToolsGui`, que no está
instalado y abría una ventana de error.

---

## 5. Movimiento fluido: protocolo stream

Con `ros_server.src` el robot paraba en cada punto: un giro de 7,7° eran 6 PTP con 5 paradas.

### Simplificación de la trayectoria
Un PTP de KUKA es una recta en espacio articular. Los puntos de MoveIt que caen sobre esa
recta no aportan nada, así que se eliminan con Ramer-Douglas-Peucker en espacio articular.

La primera versión proyectaba cada punto sobre la recta, y un zigzag de ida y vuelta sobre
la misma línea quedaba reducido a 2 puntos. Se cambió a comparar cada punto con donde
estaría según la **longitud de arco** recorrida, de modo que las vueltas atrás dan una
desviación grande y se conservan.

Resultado: un giro de un eje queda en 1 PTP.

### Cola con aproximación
`ros_stream.src`:
- **Buffer y punteros:** un buffer circular `COM_BUF[16]` con dos contadores. El PC
  escribe y avanza `COM_WR`; el robot lee y avanza `COM_RD`.
- **Sin `WAIT FOR`:** detiene el *advance run* y sin él no hay aproximación. En su lugar se
  usa `IF COM_RD < COM_WR` en un `LOOP`. Las variables globales de usuario no detienen el
  advance run.
- **Aproximación:** cada punto va con `PTP ... C_PTP`, salvo el último (`COM_LAST`), que va
  exacto.
- **Avance del robot:** `$ADVANCE = 3` permite planificar hasta 3 movimientos por delante.
- **Reinicio del programa:** al arrancar hace `COM_RD = COM_WR`, que descarta los puntos
  pendientes, y `COM_GEN + 1`, que permite al PC detectar el reinicio.

En el bridge:
- Mantiene como máximo 4 puntos sin leer por delante del robot.
- **Al cancelar** retira de la cola los no leídos. El robot termina los ya planificados.
- **Fin:** solo se da por terminado cuando el robot está en destino **y parado**. Al
  principio bastaba con estar dentro de la tolerancia, y el bridge anunciaba el final con el
  robot todavía entrando en el último punto (error de 0,425°).

### Problemas en el smartPAD
- **Teclado alemán:** `[` y `]` se escriben con AltGr+8 y AltGr+9.
- **No existe el operador `MOD`** en KRL. Daba el error "se espera `)`". Se sustituyó por
  `COM_RD - (COM_RD / 16) * 16 + 1`, que funciona porque la división de dos `INT` es entera.
- **Erratas:** `DECL INIT` en lugar de `DECL INT`, y `COM_BUF(idx)` con paréntesis en lugar
  de corchetes.

### Herramienta `kuka_vars`
Lee variables por KUKAVARPROXY sin escribir nada: modo, programa, estado, override, cola y
ejes. Sirve para comprobar el estado antes de mover.

---

## 6. Escena de colisión

[config/scene.yaml](../config/scene.yaml) contiene cajas en el frame `base_link`. Las
publica [scene_publisher.py](../kuka_bridge/scene_publisher.py) mediante el servicio
`/apply_planning_scene` de `move_group`. Cada objeto tiene su color: mesa marrón, pedestal
gris, paredes azul translúcido y suelo gris tenue.

**Medidas de la celda:**
- **Espacio de trabajo:** 0,90 m hacia atrás, y 1,10 m hacia delante y a los lados.
- **Mesa:** 46 cm (X) × 80,5 cm (Y) × 62 cm de alto desde el suelo. Está delante y
  centrada, con el borde a 50 cm del eje A1.
- **Pedestal:** 36 × 36 × 40 cm. Su cara superior está 5 mm por debajo de la base del
  robot: si se tocaran, MoveIt consideraría que el robot ya está en colisión y no
  planificaría nada.
- **Suelo:** en z = −0,40 m, por el pedestal.

Al principio se midió sin tener en cuenta el pedestal, y se corrigió bajando suelo, mesa y
paredes 40 cm.

---

## 7. Pruebas en el robot real

| Prueba | Resultado |
|---|---|
| Lectura de `/joint_states` | 20,0 Hz |
| Movimiento a home con un goal directo (single, T1) | Error 0,000° |
| MoveIt, A1 +7,7° (single, T1) | 6 PTP, correcto |
| Cuadrado de 4 esquinas (stream, T1) | Correcto, encadenado |
| Bloqueo en AUT con `allowed_modes=T1` | Rechazado sin mover |
| Cuadrado (stream, AUT, override 10 %) | Correcto, sin pulsador |
| MoveIt, A1 +8,9° (stream, AUT) | 9 puntos → 1 PTP, 3,9 s, error 0,000° |

En el simulador aislado también se probaron:
- La cancelación a mitad de trayectoria.
- El reinicio del programa durante una trayectoria.
- Un programa equivocado seleccionado.
- Trayectorias de 240°.
- Destinos fuera de los límites articulares.
- Colisiones con mesa, pedestal, suelo y paredes.

---

## 8. Decisiones

- **No mover a home al arrancar el programa KRL.** Un `PTP HOME` al inicio no pasa por
  MoveIt (no evita colisiones) ni por las validaciones del bridge, y en AUT se ejecuta al
  pulsar Start. El BCO se hace con `PTP $AXIS_ACT`, que no mueve el robot. Para ir a home se
  usa el estado `home` de MoveIt.
- **Sin límite de movimiento total por defecto.** Una vez modelada la escena, la protección
  la dan los límites articulares y las colisiones. `max_total_deg` queda disponible para
  pruebas.
- **La velocidad la decide el robot** (`VEL_PTP` × override), no MoveIt.

## 9. Siguientes pasos

1. **Gripper en la escena:** caja adjunta a `tool0`.
2. **Control del gripper:** `$OUT[n]` por KUKAVARPROXY y un servicio ROS.
3. **Pick & place:** secuencia en Python sobre MoveIt.
4. **Movimientos `LIN` cartesianos** (E6POS) en la cola, para bajadas rectas.
5. **Escalado de velocidad de MoveIt → `$VEL_AXIS`.**
