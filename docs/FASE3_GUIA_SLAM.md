# Fase 3 — SLAM: Mapeo del Ambiente (con IMU+EKF como prerequisito)

> **Objetivo:** Generar un mapa del ambiente simulado coherente con la escena de Isaac Sim, usando lidar 2D + odometría fusionada con IMU para mejor calidad.
> **Verificación final:** Mapa `.pgm`+`.yaml` guardado en `cargo_bot_navigation/maps/`. Vista del mapa en RViz2 coincide con la escena visual de Isaac.
>
> **Última actualización:** 2026-05-30 — (1) §6: `ekf.yaml` ahora fusiona yaw ABSOLUTO del IMU (ancla de heading — sin esto el /scan derivaba ~15° del mapa; ⚠️ depende de BNO085 en HW, ver memoria `cargo_bot_ekf_yaw_anchor_hw_dependency`). (2) §8: gotcha del scan count mismatch (1066 vs 1067) → nodo `scan_angle_fixer` republica `/scan`→`/scan_fixed`, `scan_topic` cambiado. (3) §8: gotcha de escena sin paredes (el lidar necesita geometría; NO rotar el lidar — orient identidad). 
>
> **Última actualización:** 2026-05-29 — §6 reescrita: `localization.launch.py` ahora levanta `robot_state_publisher` + `joint_state_publisher` + `ekf_filter_node` (decisión de la sesión 2026-05-28, motivada por el borrado completo del `ROS_TF` graph de Isaac en §4.4). §7 Test 2 y §8 Inputs actualizados en consecuencia. Agregado: edición del `setup.py` para instalar launches.

---

## Por qué este orden

Las fases 3a-3b (IMU+EKF) vienen ANTES de slam_toolbox porque la calidad del mapa depende directamente de la calidad de `odom → base_footprint`:

- **Sin IMU:** odometría es 100% wheel encoders → cualquier patinaje o desliz de rueda se acumula como drift de heading → giros cerrados desalinean el mapa, y patinajes generan saltos
- **Con IMU + EKF:** wheel odometry + gyro fusionados → heading absoluto del gyro corrige drift; acelerómetro detecta si el robot realmente se está moviendo o solo patina → mapa mucho más limpio

Mismo flujo se reutiliza en HW real con MPU6050/BNO085. Por eso vale la pena pagar el costo de setup ahora.

---

## Índice

1. [Preparar el entorno](#1-preparar-el-entorno)
2. [Agregar IMU al xacro (fase 3a)](#2-agregar-imu-al-xacro)
3. [Regenerar URDF](#3-regenerar-urdf)
4. [Crear scene_v4 con IMU OmniGraph](#4-crear-scene_v4-con-imu-omnigraph)
5. [Crear packages cargo_bot_navigation + cargo_bot_bringup](#5-crear-packages-cargo_bot_navigation--cargo_bot_bringup)
6. [Configurar EKF (robot_localization)](#6-configurar-ekf-robot_localization)
7. [Verificar IMU+EKF antes de SLAM](#7-verificar-imuekf-antes-de-slam)
8. [Configurar slam_toolbox (fase 3b)](#8-configurar-slam_toolbox)
9. [Mapear el ambiente](#9-mapear-el-ambiente)
10. [Guardar el mapa](#10-guardar-el-mapa)
11. [Verificación final end-to-end](#11-verificación-final-end-to-end)
12. [Troubleshooting](#12-troubleshooting)
13. [Tips de rendimiento](#13-tips-de-rendimiento)

---

## 1. Preparar el entorno

Antes de tocar nada de SLAM, todo el stack de Fase 2 tiene que estar funcionando.

### Checklist pre-inicio

```
[ ] Discovery Server corriendo en WSL2 (foreground)
      → bash /mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/start_discovery_server.sh

[ ] Isaac Sim lanzado con DDS configurado
      → Doble click: C:\Users\agusp\Documentos\cargo_bot_ws\config\launch_isaac_ros.cmd

[ ] scene_v3.usda abierta y en ▶ Play
      → File → Open → src/cargo_bot_simulation/scenes/scene_v3.usda

[ ] Topics esperados publicando (verificar desde WSL):
      → source /opt/ros/humble/setup.bash
      → source /mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/source_ros_wsl.sh
      → ros2 topic hz /clock     # esperar ~50 Hz
      → ros2 topic hz /odom      # esperar ~50 Hz
      → ros2 topic hz /scan      # esperar ~16 Hz
      → ros2 topic hz /tf        # esperar ~145 Hz

[ ] Instalar paquetes ROS necesarios
      → sudo apt update
      → sudo apt install ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-twist-mux
```

#### Por qué cada paquete

| Paquete | Para qué |
|---------|----------|
| `robot-localization` | El EKF que fusiona `/odom` (wheel) + `/imu/data` → `/odometry/filtered` |
| `slam-toolbox` | Algoritmo de SLAM 2D async (mapeo + loop closure) |
| `twist-mux` | Multiplexa varios publishers de `/cmd_vel` con prioridades. No urgente acá pero útil para Fase 4 — instalá ya |

> **Nota:** Si no ves `/clock` o los demás topics, volvé a la Fase 2 antes de seguir. La FASE3 asume que toda la 2 está cerrada.

---

## 2. Agregar IMU al xacro

### Qué estamos haciendo y por qué

El xacro actual tiene `imu_link` declarado como un link **vacío** (`<link name="imu_link"/>`). Para que Isaac Sim pueda colgar un OmniGraph IMU bajo este link y que el URDF sea semánticamente válido para futuros usos (`ros2_control`, MoveIt, validators), hay que:

1. Agregar `<inertial>` al link (aunque sea con masa simbólica). Sin inertial algunos importers pueden quejarse o rechazarlo silenciosamente.
2. Agregar un tag `<sensor type="imu">` análogo al `<sensor type="ray">` del lidar. Este tag es una extensión propietaria del URDF Importer de Isaac 5.1 — *a veces* honra el tag y crea el prim automático, *a veces* no (es buggy en 5.1.0). Si no lo respeta, lo agregamos manual via Tools menu en step 4.

### Comandos + snippet

**Archivo a abrir** (en tu editor):

```
C:\Users\agusp\Documentos\cargo_bot_ws\src\cargo_bot_description\urdf\sensors.xacro
```

**Localizar las líneas 37-43** (bloque actual del IMU):

```xml
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Reemplazar por:**

```xml
<!-- Isaac Sim IMU.
     Following the same pattern as the lidar <sensor> block above.
     imu_link is co-located with base_link (xyz=0 0 0) — convention for
     board-mounted IMUs at center of robot. URDF Importer in 5.1 may or
     may not honor the <sensor type="imu"> tag — if Isaac doesn't auto-create
     the IMU prim, we add it manually via Tools menu (see section 4).
     Reference: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html
     -->
<link name="imu_link">
    <xacro:inertial_box mass="0.01" x="0.02" y="0.02" z="0.005"/>
</link>

<joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<sensor name="imu" type="imu" update_rate="100">
    <parent link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</sensor>
```

### Desglose del snippet

#### `<link name="imu_link">`

| Pieza | Qué hace |
|-------|----------|
| `name="imu_link"` | Nombre único del link. Coincide con `frame_id` que usaremos al publicar mensajes IMU |
| `<xacro:inertial_box .../>` | Macro de xacro (definida en `inertial_macros.xacro`). **Self-closing, sin bloque interno** — esa es la única forma soportada. La firma es `params="mass x y z"` (sólo 4 atributos). Pasar `<origin>` adentro provoca `Unused block "origin"`; ver patrón canónico en `chassis.xacro:22` y `wheels.xacro:21,50` |
| `mass="0.01"` | 10 g. Masa típica de placa MPU6050/BNO085 real |
| `x="0.02" y="0.02" z="0.005"` | Dimensiones de la caja en metros (2cm x 2cm x 5mm) — placa chica |

El `<inertial>` que el macro expande internamente ya viene con `<origin>` implícito en `(0,0,0)`. Como el IMU está co-located con `base_link`, no hace falta override.

#### `<joint name="imu_joint" type="fixed">`

| Pieza | Qué hace |
|-------|----------|
| `type="fixed"` | El IMU está atornillado, no se mueve respecto a `base_link` |
| `<parent>` / `<child>` | base_link es el padre, imu_link el hijo |
| `<origin xyz="0 0 0">` | Posición del IMU respecto al origen de `base_link` (centro del robot). En HW real ajustás esto según donde quede en la PCB |

#### `<sensor name="imu" type="imu" update_rate="100">`

| Pieza | Qué hace |
|-------|----------|
| `type="imu"` | Le dice al URDF Importer de Isaac que cree un IMU sensor prim |
| `update_rate="100"` | 100 Hz, frecuencia típica de un MPU6050 default |
| `<parent link="imu_link"/>` | Sobre qué link va el sensor (redundante semánticamente pero el formato lo requiere) |

### Verificación

Antes de seguir, validá que el xacro compila:

```bash
# En WSL
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_description/urdf
xacro cargo_bot.urdf.xacro > /tmp/test.urdf
```

Si no tira error, está OK. Si tira `xacro:inertial_box undefined`, verificá que `inertial_macros.xacro` esté incluido en `cargo_bot.urdf.xacro` con `<xacro:include filename="inertial_macros.xacro"/>`.

---

## 3. Regenerar URDF

### Qué estamos haciendo y por qué

Isaac Sim no entiende xacro — solo URDF puro. Hay que pre-procesar.

### Comandos

```bash
# En WSL
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws
colcon build --packages-select cargo_bot_description
source install/setup.bash

# Generar URDF puro
xacro src/cargo_bot_description/urdf/cargo_bot.urdf.xacro > /tmp/cargo_bot.urdf

# Validar
check_urdf /tmp/cargo_bot.urdf

# Convertir package:// a absolute paths (Isaac no entiende package://)
sed 's|package://cargo_bot_description|/mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_description|g' /tmp/cargo_bot.urdf > /tmp/cargo_bot_isaac.urdf

# Copiar a Windows-visible path
cp /tmp/cargo_bot_isaac.urdf /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_description/urdf/cargo_bot_isaac.urdf
```

### Desglose

| Comando | Qué hace |
|---------|----------|
| `colcon build --packages-select cargo_bot_description` | Builds solo el package que tocamos. Sin `--packages-select` rebuilds todo |
| `xacro ... > /tmp/cargo_bot.urdf` | Procesa xacro: expande macros, resuelve variables, fusiona includes → URDF puro |
| `check_urdf` | Valida: links conectados, joints bien definidos, sin loops. Muestra el TF tree si está bien |
| `sed 's\|package://...\|...\|g'` | Reemplaza referencias `package://` por path absoluto. Isaac Sim corre en Windows nativo, no entiende `package://` |
| `cp ... cargo_bot_isaac.urdf` | Copia a path visible desde Windows (el `/mnt/c/...` desde WSL es `C:\...` desde Windows) |

### Verificación

```bash
ls -la /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_description/urdf/cargo_bot_isaac.urdf
```

Debería tener fecha de modificación de hace segundos.

---

## 4. Crear scene_v4 con IMU OmniGraph

### Qué estamos haciendo y por qué

Partimos de `scene_v3.usda` (que ya tiene lidar + odom + tf + cmd_vel funcionando) y agregamos el IMU. **No editamos scene_v3 directamente** — la dejamos como fallback conocido-funcionando. Si scene_v4 rompe, abrimos scene_v3.

### Pasos en Isaac Sim

#### 4.1 Save As scene_v4

1. **File → Open** → `src/cargo_bot_simulation/scenes/scene_v3.usda`
2. **File → Save As** → `src/cargo_bot_simulation/scenes/scene_v4.usda`

#### 4.2 Verificar que imu_link existe en el Stage (NO re-importar URDF)

##### ¿No tendríamos que re-importar el URDF?

**No, y conviene entender por qué:** los cambios que hicimos al xacro en la sección 2 (agregar `<inertial>` al imu_link + tag `<sensor type="imu">`) NO requieren re-import en este caso particular. Las razones:

1. **El `imu_link` ya existe en scene_v3** como Xform vacío (porque la declaración `<link name="imu_link"/>` ya estaba en el xacro original). Solo nos falta colgarle el OmniGraph del IMU bajo ese Xform que ya está.
2. **El `<inertial>` que agregamos no afecta nada físico** — Isaac importa con `merge_fixed_joints=True` por default, lo que mergea `imu_link` en `base_link`. La inertial se descarta. La razón de agregarla es documental y forward-compat.
3. **El `<sensor type="imu">` tag es buggy en Isaac 5.1.0 GA.** No confiamos en que cree el IMU prim. En su lugar, en la sección 4.3 lo creamos via script Python (`add_imu.py`) — flujo deterministic, igual al patrón que usamos en Fase 2 con `add_lidar.py`.

**El riesgo de re-importar:** borra TODO el laburo manual de scene_v3 (OmniGraph del lidar RTX, OmniGraph de cmd_vel, Articulation Root + drives, cleanup del Camera fallback). Por eso preservamos scene_v3 → scene_v4 vía Save As, no via re-import.

**Cuándo SÍ habría que re-importar:**
- Agregaste un link NUEVO que no existía en el xacro original
- Cambiaste la geometría de meshes
- Cambiaste axis/limits de joints
- Cambiaste masa/inercia de un link NO-fixed-merged

Ninguno aplica a Fase 3.

##### Verificación visual

En el Stage panel (izquierda), expandir `/World/cargo_bot/`. Debería existir `imu_link` como Xform hijo de `base_link` (heredado de scene_v3).

- ✅ Si existe: seguí al 4.3 (caso esperado)
- ❌ Si por algún motivo NO existe (raro): la opción más segura es agregar manualmente un Xform vacío llamado `imu_link` como hijo de `base_link` desde el Stage panel (click derecho → Create → Xform). NO re-importes para no romper scene_v4.

#### 4.3 Agregar IMU sensor + OmniGraph (via script Python)

##### ¿Por qué NO Tools menu?

A diferencia del Lidar RTX (que tiene auto-builder en `Tools → Robotics → ROS 2 OmniGraphs → RTX Lidar`), el IMU **NO tiene auto-builder en ROS 2 OmniGraphs** en Isaac 5.1 GA. NVIDIA solo agregó auto-builders para los sensores que requieren setup complejo (render products, AOVs). El IMU es bastante simple para no haberlo automatizado.

Las dos opciones manuales:
- **A)** Armar en Action Graph editor → Isaac 5.1 GA crashea con frecuencia armando grafos nuevos (lección de Fase 2 cuando armamos cmd_vel)
- **B)** Script Python con `og.Controller.edit` API → idempotente, deterministic, lo que recomendamos

Usamos B con el script `src/cargo_bot_simulation/scripts/setup/add_imu.py` (sigue el mismo patrón que `setup/add_lidar.py` y `setup/build_cmd_vel_graph.py` de Fase 2).

##### El gotcha del `imuPrim` target attribute

El script existe principalmente porque hay un quirk en la API del OmniGraph que **silently fails** si lo hacés mal:

El input `imuPrim` del nodo `IsaacReadIMU` NO es un string normal — es un **target attribute** (un puntero a otro prim del USD). Setearlo requiere:

```python
og.Controller.set(
    og.Controller.attribute(f"{GRAPH_PATH}/ReadIMU.inputs:imuPrim"),
    [usdrt.Sdf.Path(IMU_PRIM_PATH)],   # ← lista de usdrt.Sdf.Path
)
```

Si lo pasás como string via `keys.SET_VALUES: [("ReadIMU.inputs:imuPrim", "/World/...")]`, **no tira error pero el publisher publica IMU readings = 0 todo el tiempo**. Es exactamente el tipo de bug que después tomarías horas en debuggear. El script lo hace bien.

##### Pasos para correr el script

1. En Isaac Sim, con scene_v4 abierta (y **NO en Play** todavía):
2. **Window → Script Editor**
3. **File → Open...** → seleccionar `C:\Users\agusp\Documentos\cargo_bot_ws\src\cargo_bot_simulation\scripts\setup\add_imu.py`
4. Click **Run** (o `Ctrl+Enter`)

Output esperado en el Script Editor console:

```
[add_imu] step 0a stage acquired
[add_imu] step 0b parent /World/cargo_bot/imu_link OK
[add_imu] step 1 created IsaacImuSensor at /World/cargo_bot/imu_link/imu_sensor
[add_imu] step 2 created graph /World/cargo_bot/imu_graph with 5 nodes
[add_imu]   node: /World/cargo_bot/imu_graph/OnPlaybackTick
[add_imu]   node: /World/cargo_bot/imu_graph/ROS2Context
[add_imu]   node: /World/cargo_bot/imu_graph/ReadSimTime
[add_imu]   node: /World/cargo_bot/imu_graph/ReadIMU
[add_imu]   node: /World/cargo_bot/imu_graph/PublishIMU
[add_imu] step 3 bound ReadIMU.imuPrim -> /World/cargo_bot/imu_link/imu_sensor
[add_imu] DONE.
```

##### Qué quedó creado en el Stage

Después del script, en el Stage panel deberías ver:

```
/World/cargo_bot/
├── imu_link/
│   └── imu_sensor          ← IsaacImuSensor prim (nuevo)
├── imu_graph/              ← Action Graph (nuevo)
│   ├── OnPlaybackTick      (omni.graph.action.OnPlaybackTick)
│   ├── ROS2Context         (isaacsim.ros2.bridge.ROS2Context)
│   ├── ReadSimTime         (isaacsim.core.nodes.IsaacReadSimulationTime)
│   ├── ReadIMU             (isaacsim.sensors.physics.IsaacReadIMU)
│   └── PublishIMU          (isaacsim.ros2.bridge.ROS2PublishImu)
└── ... (resto de scene_v4 intacto)
```

##### Diagrama del grafo que arma el script

```
OnPlaybackTick ──► IsaacReadIMU(imuPrim=/World/.../imu_sensor) ──► ROS2PublishImu
                                                                   (topic=imu/data,
                                                                    frameId=imu_link)
                          paralelo:
                          IsaacReadSimulationTime ──► ROS2PublishImu.timeStamp
                          ROS2Context ──► ROS2PublishImu.context
```

##### Qué es un OmniGraph (concepto general)

Antes de explicar los nodos, vale la pena entender qué es OmniGraph:

OmniGraph es el **sistema de visual scripting** de Isaac Sim. Cada "nodo" es una función (compute kernel) con inputs y outputs tipados. Los nodos se conectan con cables (igual que Unreal Blueprints o Unity Visual Scripting). El grafo se ejecuta cada simulation frame.

Hay dos tipos de cables:
- **Exec cables** (líneas blancas/grises) — controlan el ORDEN de ejecución. Si A.execOut → B.execIn, entonces B corre DESPUÉS de A. Sin un cable exec llegándole, un nodo NUNCA se ejecuta.
- **Data cables** (líneas con color según el tipo, ej. azul para Vec3, verde para bool) — pasan VALORES. Un nodo lee los inputs antes de ejecutar, escribe los outputs cuando termina.

Todo grafo arranca con un **trigger node** (en general `OnPlaybackTick`) que se dispara cada frame de Play.

#### Desglose de cada node del IMU graph

##### Node 1: `OnPlaybackTick` (trigger)

| Item | Detalle |
|------|---------|
| Tipo | `omni.graph.action.OnPlaybackTick` |
| Inputs | Ninguno (es source del grafo) |
| Outputs | `tick` (exec) — pulsado cada simulation frame |
| Por qué | Es el latido del grafo. Sin un `OnPlaybackTick`, ningún nodo del grafo se ejecutaría jamás. Equivalente a `void Update()` en Unity o `Tick(DeltaTime)` en Unreal |

##### Node 2: `IsaacReadIMU` (compute)

| Item | Detalle |
|------|---------|
| Tipo | `isaacsim.sensors.physics.IsaacReadIMU` (en 5.1, antes en 4.x era `omni.isaac.sensor.IsaacReadIMU`) |
| Inputs | `execIn` (exec, viene del tick), `imuPrim` (TARGET attribute al prim del IMU), `readGravity` (bool), `useLatestData` (bool) |
| Outputs | `execOut` (exec), `linAcc` (Vec3 m/s²), `angVel` (Vec3 rad/s), `orientation` (Quat IJKR), `sensorTime` (float) |
| Por qué | Es la interfaz que lee el simulado físico del IMU. PhysX (motor de física de Isaac) calcula internamente las aceleraciones y velocidades angulares del rigid body al que el prim está pegado. Este nodo expone esos valores como datos consumibles |

**Detalle técnico de los outputs:**
- `linAcc` incluye la aceleración real del robot más la gravedad si `readGravity=true`. Es lo que un MPU6050 real reporta — sentís 9.81 m/s² hacia arriba cuando estás quieto.
- `angVel` son las tasas de rotación instantáneas alrededor de los tres ejes. Es lo que reporta un gyro.
- `orientation` es la orientación absoluta integrada — útil pero NO la fusionamos en el EKF (drifta sin magnetómetro).
- `sensorTime` es timestamp interno del sensor (no lo usamos, el timestamp del mensaje ROS viene de `IsaacReadSimulationTime` separado).

**Asimetría de nombres a tener presente:** los outputs del `IsaacReadIMU` (`linAcc`, `angVel`) tienen nombres distintos a los inputs del `ROS2PublishImu` (`linearAcceleration`, `angularVelocity`). El script `add_imu.py` hace el mapeo correcto en `keys.CONNECT`.

##### Node 3: `ROS2Context` (helper)

| Item | Detalle |
|------|---------|
| Tipo | `isaacsim.ros2.bridge.ROS2Context` |
| Inputs | (parámetros internos: domain ID, ROS distro) |
| Outputs | `context` (objeto opaco que representa el ROS 2 context de DDS) |
| Por qué | Cada ROS 2 process necesita inicializar un "context" — equivalente a `rclcpp::Context` en C++ o `rclpy.init()` en Python. Este nodo crea ese context compartido para que todos los publishers/subscribers de la escena lo reutilicen. Si no estuviera, cada publisher tendría que crear su propio context, ineficiente |

**Nota:** este nodo NO se conecta vía exec — el output `context` es un valor compartido que TODOS los nodos de publish/subscribe leen como input directamente.

##### Node 4: `IsaacReadSimulationTime` (helper)

| Item | Detalle |
|------|---------|
| Tipo | `isaacsim.core.nodes.IsaacReadSimulationTime` |
| Inputs | `resetOnStop` (bool) |
| Outputs | `simulationTime` (double, seconds) |
| Por qué | Provee el sim-time actual (consistente con `/clock` que publica Isaac). Lo usamos para sellar el timestamp del mensaje `sensor_msgs/Imu`. Sin esto, el publisher usaría wall-clock por default, lo que rompe consistencia bajo `use_sim_time:=true` en SLAM/EKF |

**Por qué este nodo y no `OnPlaybackTick.tick`:** el output `tick` de OnPlaybackTick es solo un pulso de ejecución, no contiene tiempo. `IsaacReadSimulationTime` lee el reloj de simulación interno de PhysX y lo expone como número.

##### Node 5: `ROS2PublishImu` (bridge)

| Item | Detalle |
|------|---------|
| Tipo | `isaacsim.ros2.bridge.ROS2PublishImu` |
| Inputs (data) | `linearAcceleration` (Vec3), `angularVelocity` (Vec3), `orientation` (Quat), `timeStamp` (double, sec) |
| Inputs (config) | `execIn`, `context`, `topicName` (string), `frameId` (string), `nodeNamespace` (string), `queueSize` (uint64), `qosProfile` (string), `publishLinearAcceleration` (bool), `publishAngularVelocity` (bool), `publishOrientation` (bool) |
| Outputs | (ninguno — escribe al bus DDS) |
| Por qué | Empaqueta los valores que leyó IsaacReadIMU en un mensaje `sensor_msgs/Imu` (con timestamp del IsaacReadSimulationTime) y lo publica al topic vía DDS. Cualquier suscriptor en WSL2 (vía Discovery Server) lo recibe |

**Detalle del mensaje `sensor_msgs/Imu`:**
- `header.stamp` = tiempo actual de simulación (sim time, no wall time)
- `header.frame_id` = lo que pongas en el parámetro `frameId`
- `orientation` + covariance 3x3
- `angular_velocity` + covariance 3x3
- `linear_acceleration` + covariance 3x3

Las covarianzas son 0 (no informadas) en sim — el EKF usa valores conservadores que vienen del `ekf.yaml`.

#### Cable connections que arma el script

| Cable | Tipo | Razón |
|-------|------|-------|
| `OnPlaybackTick.outputs:tick` → `ReadIMU.inputs:execIn` | exec | Para que el reader se ejecute cada frame |
| `ReadIMU.outputs:execOut` → `PublishIMU.inputs:execIn` | exec | El publish corre DESPUÉS de leer (mismos valores que se leyeron en el tick) |
| `ReadIMU.outputs:linAcc` → `PublishIMU.inputs:linearAcceleration` | data Vec3 | Mapeo asimétrico (`linAcc` → `linearAcceleration`) |
| `ReadIMU.outputs:angVel` → `PublishIMU.inputs:angularVelocity` | data Vec3 | Mapeo asimétrico |
| `ReadIMU.outputs:orientation` → `PublishIMU.inputs:orientation` | data Quat | Pasar el dato |
| `ReadSimTime.outputs:simulationTime` → `PublishIMU.inputs:timeStamp` | data double | Timestamp del mensaje viene del sim-time, no wall-time |
| `ROS2Context.outputs:context` → `PublishIMU.inputs:context` | data context | Publisher usa el context DDS compartido |

#### Valores que setea el script (`SET_VALUES` + `og.Controller.set` para target)

| Nodo | Atributo | Valor | Por qué |
|------|----------|-------|---------|
| `ReadIMU` | `imuPrim` ⚠️ TARGET | `[usdrt.Sdf.Path("/World/cargo_bot/imu_link/imu_sensor")]` | Vinculo al IMU sensor. **Target attribute, NO string** — esto es el gotcha principal |
| `ReadIMU` | `readGravity` | `true` | El acelerómetro reporta gravedad cuando no se mueve (~9.81 m/s² en Z) — realismo de MPU6050 |
| `ReadIMU` | `useLatestData` | `false` | Usar el snapshot del physics tick actual, no el último frame disponible |
| `PublishIMU` | `topicName` | `imu/data` | Sin `/` inicial. ROS 2 lo agrega automático |
| `PublishIMU` | `frameId` | `imu_link` | **TIENE que coincidir EXACTO con el link del TF tree**. Si no coincide, EKF no encuentra la transformada y la fusion falla silenciosamente |
| `PublishIMU` | `nodeNamespace` | (vacío) | Topic en namespace raíz |
| `PublishIMU` | `queueSize` | `10` | Buffer de mensajes pendientes |
| `PublishIMU` | `qosProfile` | (default sensor_data) | Para IMU está bien. Si CPU sufre se baja a best_effort |
| `PublishIMU` | `publishLinearAcceleration` | `true` | Incluir el campo en el msg |
| `PublishIMU` | `publishAngularVelocity` | `true` | Incluir el campo |
| `PublishIMU` | `publishOrientation` | `true` | Incluir el campo |
| `ROS2Context` | `useDomainIDEnvVar` | `true` | Usar `ROS_DOMAIN_ID` del environment (`=1` en nuestro stack) |
| `ReadSimTime` | `resetOnStop` | `false` | Mantener monotonic sim-time entre Play/Stop |

#### 4.4 Limpieza crítica de TF — `odom → base_footprint`

⚠️ **Esto es lo más sutil de toda la sesión.** Léelo dos veces.

##### Concepto: cómo funciona TF en ROS 2

TF (Transform Library) mantiene un **árbol de transformaciones** entre frames de referencia. Cada nodo del árbol es un frame (ej. `map`, `odom`, `base_footprint`, `lidar_link`). Cada arista del árbol es una transformación (translación + rotación) entre frame padre e hijo.

Reglas clave:
1. Cada frame tiene **exactamente UN padre** (excepto el root, que no tiene)
2. Cada transformación `A → B` (de padre A a hijo B) la **publica UN ÚNICO nodo** — quien sea el "dueño" de esa relación
3. Múltiples publishers de la misma transformación = **conflicto** indeterminista. RViz hace un mejor esfuerzo pero el resultado es impredecible (a veces gana uno, a veces el otro, los timestamps no alinean)

##### El conflicto que estamos resolviendo

Actualmente scene_v3 tiene un OmniGraph de Odometry que hace **dos cosas:**

1. Publica `/odom` (topic — un `nav_msgs/Odometry` con la pose+twist actual del robot) — esto QUEREMOS mantener, porque el EKF lo usa como input
2. Publica la TF `odom → base_footprint` (vía un nodo `RawTransformTree` adicional) — esto NO queremos, porque el EKF va a ser la única fuente de esa TF cuando lo metamos

Si dejamos ambas fuentes:
- Isaac publica `odom → base_footprint` con la pose RAW del wheel odom (drifta con el patinaje)
- EKF publica `odom → base_footprint` con la pose FILTRADA por gyro

RViz va a mostrar warnings tipo `Frame odom in TF tree has multiple parents` (mentira, tiene un padre — el problema es múltiples publishers del mismo arco) y el sistema se vuelve indeterminista. **Por lo tanto: hay que SACAR uno de los dos.** Sacamos el de Isaac porque queremos la pose filtrada del EKF.

#### Cómo identificar la rama que publica la TF

En el Stage, encontrar el OmniGraph de Odometry. Debe haber un nodo `RawTransformTree` o `ROS2PublishRawTransformTree` conectado al output del `IsaacComputeOdometry`.

#### Cómo desactivar SOLO esa rama

Opción A (más segura): **Desconectar el cable de exec** entre `IsaacComputeOdometry` → `RawTransformTree`. Eso deja el grafo intacto pero la TF no se publica.

Opción B: **Eliminar el nodo `RawTransformTree`** del grafo. Cleanup más limpio pero menos reversible.

**Mantener intacto:**
- `IsaacComputeOdometry` (necesario, computa la odometría)
- `ROS2PublishOdometry` (publica el topic `/odom`)
- El `ROS2PublishTransformTree` separado que publica `base_link → wheels` (ese sigue siendo necesario)

#### Verificación visual

Después de desconectar/borrar, el grafo de Odometry debería:
- Seguir teniendo el path: `OnPlaybackTick → IsaacComputeOdometry → ROS2PublishOdometry` (publica topic)
- NO tener ramificación a `RawTransformTree` para `odom → base_footprint`

#### 4.5 Save + Play

1. **File → Save** (sobrescribe scene_v4.usda)
2. **▶ Play**

#### Verificación inmediata

En WSL:

```bash
ros2 topic hz /imu/data
```

Debe mostrar ~100 Hz. Si timeout, el OmniGraph del IMU no está publicando — leé Troubleshooting sección 12.

---

## 5. Crear packages cargo_bot_navigation + cargo_bot_bringup

### Qué estamos haciendo y por qué

Hasta ahora solo existía `cargo_bot_description`. Para SLAM + Nav2 necesitamos dos packages nuevos según el `cargo_bot_ws/src/` layout del MASTER_PLAN sección 5:

- **`cargo_bot_navigation`** (ament_cmake): contiene **configs** (YAMLs de EKF, SLAM, Nav2) y artefactos (mapas). No tiene código C++/Python por ahora.
- **`cargo_bot_bringup`** (ament_python): contiene **launch files** que orquestan nodos de distintos packages.

Separar config de launches es convención ROS 2 estándar — facilita reutilizar configs entre distintos launches (sim vs hardware real).

### Comandos

```bash
# En WSL
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src

# Package 1: configs y mapas
ros2 pkg create --build-type ament_cmake cargo_bot_navigation \
  --dependencies rclcpp nav2_common

# Package 2: launches
ros2 pkg create --build-type ament_python cargo_bot_bringup
```

### Desglose

| Token | Qué hace |
|-------|----------|
| `ros2 pkg create` | Cmdlet de ROS 2 que crea estructura mínima de un package |
| `--build-type ament_cmake` | Para packages C++ o que tienen archivos a instalar (configs, launches). Genera `CMakeLists.txt` + `package.xml` |
| `--build-type ament_python` | Para packages Python puros. Genera `setup.py` + `package.xml` |
| `--dependencies rclcpp nav2_common` | Pre-agrega estas deps al `package.xml`. `rclcpp` = librería C++ ROS 2 (no la usamos directo acá pero queda lista). `nav2_common` = utilities de Nav2 |
| `cargo_bot_navigation` (positional) | Nombre del package, en snake_case |

### Estructura post-creación

Después de los comandos:

```
src/
├── cargo_bot_navigation/
│   ├── CMakeLists.txt        ← editable
│   ├── package.xml           ← editable
│   └── include/, src/        ← se borran (no usamos)
└── cargo_bot_bringup/
    ├── setup.py              ← editable
    ├── setup.cfg
    ├── package.xml           ← editable
    ├── resource/cargo_bot_bringup
    ├── cargo_bot_bringup/__init__.py
    └── test/
```

### Crear estructura de carpetas adicional

```bash
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_navigation
mkdir -p config maps
rm -rf include src   # No usamos C++ por ahora

cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_bringup
mkdir -p launch
```

### Editar `cargo_bot_navigation/CMakeLists.txt`

Reemplazar el contenido por:

```cmake
cmake_minimum_required(VERSION 3.8)
project(cargo_bot_navigation)

find_package(ament_cmake REQUIRED)

# Instalar configs y maps para que sean accesibles via $(find-pkg-share)
install(DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

#### Desglose

| Línea | Qué hace |
|-------|----------|
| `cmake_minimum_required` | Versión mínima de CMake requerida |
| `project(cargo_bot_navigation)` | Define el nombre del package en CMake |
| `find_package(ament_cmake REQUIRED)` | Trae las macros de ROS 2 (ament) |
| `install(DIRECTORY config maps ...)` | Copia las carpetas `config/` y `maps/` a `install/cargo_bot_navigation/share/cargo_bot_navigation/`. Esto las hace accesibles desde launches via `$(find-pkg-share cargo_bot_navigation)/config/ekf.yaml` |
| `ament_package()` | Cierra la definición del package |

---

## 6. Configurar EKF (robot_localization)

### Qué estamos haciendo y por qué

`robot_localization` es un EKF (Extended Kalman Filter) que fusiona múltiples fuentes de odometría/IMU/GPS y produce una **estimación de pose y velocidad más robusta** que cualquiera de las fuentes individuales.

Lo configuramos para fusionar:
- `/odom` (de Isaac, wheel encoders) → tomamos solo velocidades, NO la posición integrada (que drifta)
- `/imu/data` (de Isaac, gyro+accel) → tomamos yaw rate y aceleración linear

Output: `/odometry/filtered` + publica la TF `odom → base_footprint` (única fuente de esa TF).

### Crear `cargo_bot_navigation/config/ekf.yaml`

Crear el archivo con este contenido:

```yaml
ekf_filter_node:
  ros__parameters:
    # Frequency at which the EKF re-estimates state and publishes.
    # 30 Hz is a good balance between resolution and CPU.
    frequency: 30.0

    # 2D mode collapses pitch, roll, z to zero — appropriate for diff-drive on flat floor.
    # Reduces dimensionality of the filter from 15-dim to 9-dim → faster + more stable.
    two_d_mode: true

    # Publish the filtered odom → base_footprint TF. EKF is the SINGLE authority
    # for this TF — that's why we disconnected the Isaac OmniGraph's TF publish
    # in section 4.4.
    publish_tf: true

    # Frame conventions: REP-105
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom    # Fusion en frame odom (no map todavía, eso es AMCL en Fase 4)

    use_sim_time: true   # respect /clock from Isaac

    # ─── Input 0: wheel odometry from Isaac ─────────────────────────────────
    odom0: odom
    # Vector of booleans indicating which state variables to FUSE from this input.
    # Order: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    # We fuse ONLY vx and vyaw — the position integrated from wheel odometry drifts.
    # The EKF integrates velocities itself, more robustly than using odom's pose.
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false

    # ─── Input 1: IMU from Isaac ─────────────────────────────────────────────
    imu0: imu/data
    # Fuse ABSOLUTE yaw (index 5 = true) + yaw rate + linear accel x.
    # ACTUALIZADO 2026-05-30: el yaw absoluto es el ANCLA DE HEADING. Sin él, el
    # filtro dead-reckonea la orientación integrando vyaw y DERIVA (~15° por
    # sesión observado -> el /scan en vivo se veía rotado respecto al mapa de
    # slam, aunque las distancias del lidar eran correctas). En Isaac el IMU da
    # orientación ground-truth (no deriva); en HW real un BNO085 da lo mismo.
    # ⚠️ DEPENDENCIA HW: con MPU6050 (sin magnetómetro) el yaw deriva -> volver
    #    índice 5 a false y anclar heading con SLAM/AMCL. Ver memoria
    #    cargo_bot_ekf_yaw_anchor_hw_dependency.
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true   # IMU reports g when stationary, remove it

    # ─── Process noise (how much we trust the model vs sensors) ──────────────
    # Diagonal matrix. Bigger values = more trust in sensors, less in model.
    # These are conservative defaults; tune later if filter is too sluggish or too jumpy.
    # NOTA: TODOS los valores son `0.0` (no `0`). El parser YAML de ROS 2 es estricto
    #       con tipos en arrays — un solo `0` (int) mezclado con `0.05` (float)
    #       hace fallar el load con "Sequence should be of same type".
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.025, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]
```

### El vector de estado 15D del EKF (concepto crítico)

robot_localization mantiene internamente un **vector de estado de 15 dimensiones** que representa todo lo que sabe del robot en cada instante:

| Índice | Dimensión | Símbolo | Unidad |
|--------|-----------|---------|--------|
| 0 | Posición X | x | m |
| 1 | Posición Y | y | m |
| 2 | Posición Z | z | m |
| 3 | Roll (rotación alrededor X) | roll | rad |
| 4 | Pitch (rotación alrededor Y) | pitch | rad |
| 5 | Yaw (rotación alrededor Z) | yaw | rad |
| 6 | Velocidad linear X | vx | m/s |
| 7 | Velocidad linear Y | vy | m/s |
| 8 | Velocidad linear Z | vz | m/s |
| 9 | Velocidad angular roll | vroll | rad/s |
| 10 | Velocidad angular pitch | vpitch | rad/s |
| 11 | Velocidad angular yaw | vyaw | rad/s |
| 12 | Aceleración linear X | ax | m/s² |
| 13 | Aceleración linear Y | ay | m/s² |
| 14 | Aceleración linear Z | az | m/s² |

**`two_d_mode: true` colapsa las dimensiones [z, roll, pitch, vz, vroll, vpitch, az] a cero** porque el robot diff-drive en piso plano no se mueve en esas dimensiones. Esto baja la complejidad del filtro de 15D a 9D efectivos — más rápido + más estable.

### Cómo se leen los arrays `*_config`

Cuando ves `odom0_config: [false, false, false, false, false, false, true, false, false, false, false, true, false, false, false]`, cada `true`/`false` corresponde a una dimensión del vector arriba (en mismo orden):

```
Posición:        [x=false, y=false, z=false]
Orientación:     [roll=false, pitch=false, yaw=false]
Velocidad lin:   [vx=TRUE,  vy=false, vz=false]
Velocidad ang:   [vroll=false, vpitch=false, vyaw=TRUE]
Aceleración:     [ax=false, ay=false, az=false]
```

Donde dice `true`, el EKF **fusiona ese campo** del mensaje de entrada.
Donde dice `false`, ignora ese campo del mensaje (lo deja para que otros sensores lo aporten, o lo integra internamente).

### Por qué fusionamos solo lo que fusionamos

#### Para `odom0` (wheel odometry de Isaac): vx + vyaw

- **NO fusionamos posición** (x, y, yaw absolutos) porque la posición que reporta wheel odom es **integrada** internamente — cada patinaje, cada error de calibración del wheel radius, se acumula como drift. El EKF integra mejor por su cuenta.
- **SÍ fusionamos velocidades** (vx, vyaw) porque son los valores INSTANTÁNEOS calculados directamente de los encoders. Mucho más confiables.

#### Para `imu0` (IMU de Isaac): vyaw + ax

- **NO fusionamos orientación absoluta** (yaw) porque sin magnetómetro, el yaw del IMU drifta con el tiempo. Un MPU6050 real puede driftar varios grados por minuto.
- **SÍ fusionamos yaw rate** (vyaw) porque es lo que reporta el gyro directamente — muy confiable corto plazo.
- **SÍ fusionamos aceleración linear X** (ax) porque permite detectar cuándo el robot está realmente moviéndose vs. patinando. Si las ruedas giran (`/odom` reporta vx=0.5) pero el accel reporta ax≈0, el EKF infiere que el robot NO se está moviendo realmente.

### Desglose completo de todos los parámetros del EKF

| Parámetro | Valor | Por qué |
|-----------|-------|---------|
| `frequency` | 30.0 | Output rate. 3× la del scan, suficiente para Nav2. Más alto = más CPU sin ganancia |
| `two_d_mode` | true | Colapsa pitch/roll/z. Apropiado para diff-drive en piso plano |
| `publish_tf` | true | EKF es la ÚNICA fuente de `odom → base_footprint` ahora |
| `map_frame` | map | Frame del mapa global (lo usará AMCL en Fase 4) |
| `odom_frame` | odom | Frame de odometría continua |
| `base_link_frame` | base_footprint | Frame del robot. Usamos base_footprint (en el piso, sin altura) en lugar de base_link (centro del robot) porque es lo que Nav2 espera |
| `world_frame` | odom | En qué frame fusionar. En Fase 3 fusionamos en `odom` (filtro local). En Fase 4, AMCL agregará `map → odom` |
| `use_sim_time` | true | Respetar /clock de Isaac. Sin esto, timestamps no alinean |
| `odom0` | odom | Topic del primer input (wheel odom) |
| `odom0_config` | (ver array arriba) | Solo vx y vyaw del wheel odom |
| `odom0_queue_size` | 10 | Cuántos mensajes bufferear antes de procesar. 10 está bien para 50 Hz input |
| `odom0_differential` | false | true integraría velocidades en lugar de usar la pose. Como no fusionamos pose, no aplica |
| `odom0_relative` | false | true normalizaría poses al frame del primer mensaje. No queremos |
| `imu0` | imu/data | Topic del segundo input (IMU) |
| `imu0_config` | (ver array arriba) | Solo vyaw y ax del IMU |
| `imu0_queue_size` | 10 | Buffer |
| `imu0_differential` | false | Same as odom |
| `imu0_relative` | false | Same as odom |
| `imu0_remove_gravitational_acceleration` | true | Resta g de Z. CRÍTICO — sin esto el EKF cree que el robot acelera hacia arriba a 9.81 m/s² siempre |

### `process_noise_covariance` — la matriz de ruido del proceso

Es una matriz 15×15 diagonal (los valores fuera de la diagonal son 0). Cada valor de la diagonal representa **cuánto ruido asumimos en cada dimensión del estado por unidad de tiempo**.

Lectura intuitiva:
- **Valor chico** (ej. 0.01) = "este estado es muy estable, no cambia mucho entre updates"
- **Valor grande** (ej. 0.1) = "este estado puede variar bastante, dale más libertad al filtro"

El EKF usa estos valores como prior — si una observación es muy distinta del estado actual, decide cuánto creerle:
- Si process_noise alto en esa dim → cree más a la observación (porque el modelo es ruidoso)
- Si process_noise bajo en esa dim → cree menos a la observación (porque el modelo es confiable)

Los defaults que dejo son conservadores y andan bien para sim. **Si después de testear veés:**
- **Filtro sluggish** (responde lento a giros, "rinde" tarde) → subir valores de yaw y vyaw
- **Filtro jumpy** (pose salta) → bajar valores de x, y

Diagonal del default:
```
x, y, z:               0.05, 0.05, 0.06
roll, pitch, yaw:      0.03, 0.03, 0.06
vx, vy, vz:            0.025, 0.025, 0.04
vroll, vpitch, vyaw:   0.01, 0.01, 0.02
ax, ay, az:            0.01, 0.01, 0.015
```

### Crear `cargo_bot_bringup/launch/localization.launch.py`

El launch arma TRES nodos en una sola descripción:

1. **`robot_state_publisher`** — autoridad del subtree URDF (`base_footprint → base_link → wheels/lidar/imu`). Reemplaza al `ROS_TF` graph de Isaac que borramos en §4.4.
2. **`joint_state_publisher`** — publica `/joint_states` con todos los joints non-fixed en pos=0. Sin esto, `robot_state_publisher` no puede calcular las transforms de los wheels (joints non-fixed) y RViz se queja `No transform from [left_wheel_link]`.
3. **`ekf_filter_node`** — fusiona `/odom` + `/imu/data` → `/odometry/filtered` + TF `odom → base_footprint`.

Crear el archivo:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path al xacro instalado por cargo_bot_description
    xacro_path = PathJoinSubstitution([
        FindPackageShare('cargo_bot_description'),
        'urdf',
        'cargo_bot.urdf.xacro'
    ])

    # robot_description = output de `xacro <xacro_path>` (string URDF puro).
    # ParameterValue(..., value_type=str) le dice a ROS 2 "es string, no parsees como YAML".
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # Path al ekf.yaml instalado por cargo_bot_navigation
    ekf_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_navigation'),
        'config',
        'ekf.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Isaac simulation clock (/clock topic)'
        ),

        # ── 1. robot_state_publisher ──
        # Lee robot_description, publica /tf_static (joints fixed: imu, lidar, caster)
        # y /tf (joints non-fixed: wheels, recalculadas a partir de /joint_states).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # ── 2. joint_state_publisher ──
        # Publica /joint_states con los wheels en pos=0. No giran visualmente
        # en RViz, pero SLAM/Nav2 no leen las wheels: usan base_footprint y
        # lidar_link (joints fixed), que rsp publica como /tf_static.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # ── 3. ekf_filter_node ──
        # name='ekf_filter_node' DEBE coincidir EXACTO con el primer key del
        # ekf.yaml. Si no, los params se ignoran silenciosamente y el filtro
        # arranca con defaults.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
```

### Actualizar `cargo_bot_bringup/setup.py` para instalar los launches

`ros2 pkg create --build-type ament_python` NO instala launches automáticamente — hay que decírselo. Abrir `src/cargo_bot_bringup/setup.py`. Llega con esta forma:

```python
from setuptools import find_packages, setup

package_name = 'cargo_bot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # ... resto del setup() ...
```

Hay que tocar **dos cosas**:

(a) **Agregar dos imports arriba de todo** (`import os` y `from glob import glob`):

```python
import os
from glob import glob

from setuptools import find_packages, setup
```

(b) **Agregar una entrada al `data_files`** para que `colcon build` copie los launches a `install/`:

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
```

#### Por qué hay que hacer esto en ament_python pero no en ament_cmake

- **`ament_cmake`**: el `install(DIRECTORY launch ...)` en `CMakeLists.txt` copia carpetas completas. Por eso `cargo_bot_description` instala `urdf/` y `meshes/` automáticamente.
- **`ament_python`**: usa `setuptools` puro. `setup.py` solo instala lo listado en `data_files`. Si no aparece, no viaja al `install/`.

**Gotcha común:** olvidarse el `from glob import glob` produce `NameError: name 'glob' is not defined` al build, con un stack trace de `colcon` largo y confuso. El error real está dos niveles abajo.

### Cómo funciona el sistema de launch de ROS 2 (conceptos)

Los `*.launch.py` no son scripts normales — son **declaraciones** de qué nodos lanzar, qué params pasarles, en qué orden, con qué dependencias. ROS 2 los ejecuta diferido (lazy) usando "substitutions" que se resuelven en runtime.

#### `LaunchConfiguration('foo')`

Es una **variable diferida**. Cuando hacés `ros2 launch ... use_sim_time:=true`, el valor `"true"` no se inyecta al instante — `LaunchConfiguration('use_sim_time')` queda como un placeholder que se resuelve cuando el sistema realmente lanza el nodo.

#### `DeclareLaunchArgument('foo', default_value='bar')`

Declara un arg que el launch acepta vía CLI. Si el usuario no pasa `foo:=...`, queda en el default. **Tiene que estar declarado** antes de usarse, si no `LaunchConfiguration('foo')` lee vacío.

#### `FindPackageShare('paquete')`

Resuelve en runtime al path `install/paquete/share/paquete/`. **Esto es crítico** porque el path de instalación no se conoce hasta que el workspace está sourceado — puede ser `~/cargo_bot_ws/install/...`, `/opt/ros/.../...`, dependiendo de cómo se buildó.

#### `PathJoinSubstitution([part1, part2, ...])`

Concatena partes para formar un path. Cada parte puede ser un string o otra substitution (como `FindPackageShare`). Resuelve en runtime el path completo.

Ejemplo: `PathJoinSubstitution([FindPackageShare('cargo_bot_navigation'), 'config', 'ekf.yaml'])` se resuelve a algo como `/mnt/c/Users/agusp/Documentos/cargo_bot_ws/install/cargo_bot_navigation/share/cargo_bot_navigation/config/ekf.yaml`.

#### `Node(package='...', executable='...', name='...', parameters=[...])`

Esta es la acción que efectivamente lanza el nodo. Equivalente a `ros2 run package executable --ros-args ...` pero declarativo.

| Argumento | Qué hace |
|-----------|----------|
| `package` | Nombre del package donde vive el ejecutable |
| `executable` | Nombre del binario o script (lo que normalmente correrías con `ros2 run`) |
| `name` | Nombre RUNTIME del nodo. **CRÍTICO**: tiene que COINCIDIR EXACTO con el primer key del YAML de params, sino los params no se aplican |
| `output='screen'` | Manda stdout/stderr a la terminal. Default es `log`, que va a archivo |
| `parameters=[...]` | Lista de fuentes de parámetros. Cada item puede ser un path a YAML o un dict Python. ROS 2 los fusiona en orden — el último gana en caso de conflicto |
| `remappings=[...]` | Remapeos de topics. Ej. `('cmd_vel', 'robot/cmd_vel')` cambia el topic visible |

### Desglose línea por línea del `localization.launch.py`

| Línea | Qué hace |
|-------|----------|
| `use_sim_time = LaunchConfiguration('use_sim_time')` | Crea la variable diferida |
| `xacro_path = PathJoinSubstitution([FindPackageShare('cargo_bot_description'), 'urdf', 'cargo_bot.urdf.xacro'])` | Path al xacro instalado. `cargo_bot_description` ya instala `urdf/` vía su `CMakeLists.txt` |
| `Command(['xacro ', xacro_path])` | Ejecuta `xacro <path>` en RUNTIME y captura el stdout. El espacio después de `'xacro '` es necesario porque `Command` concatena tokens sin separador — sin él quedaría `xacro/path/...` |
| `ParameterValue(Command(...), value_type=str)` | Marca el parámetro como string. Sin esto, ROS 2 intenta parsear el XML del URDF como YAML y muere |
| `ekf_config = PathJoinSubstitution([FindPackageShare('cargo_bot_navigation'), 'config', 'ekf.yaml'])` | Path al YAML de EKF |
| `DeclareLaunchArgument('use_sim_time', default_value='true', ...)` | Declara el arg CLI con default `true` |
| `Node(package='robot_state_publisher', ...)` | Nodo 1: lee `robot_description`, publica `/tf_static` (todas las transforms fixed: imu/lidar/caster) y `/tf` (transforms de joints non-fixed) |
| `Node(package='joint_state_publisher', ...)` | Nodo 2: publica `/joint_states` con todos los wheels en pos=0. Sin esto, rsp no puede calcular las TF de wheels y RViz se queja |
| `Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', ...)` | Nodo 3: lanza `ekf_node`. **`name='ekf_filter_node'` debe COINCIDIR con el primer key del YAML** (`ekf_filter_node:` línea 1 del `ekf.yaml`). Si no coincide, ROS 2 no encuentra los params y el nodo usa defaults silenciosamente |
| `parameters=[ekf_config, {'use_sim_time': use_sim_time}]` | Dos fuentes: YAML completo + dict con override. El dict gana en caso de duplicado (use_sim_time aparece en ambos, valores coinciden) |

### Build + launch

Importante: si modificaste `sensors.xacro` en §2 (el upgrade del `imu_link`) pero todavía no rebuildeaste `cargo_bot_description`, el xacro que `robot_state_publisher` va a leer en runtime sigue siendo la versión vieja. Hay que rebuild también ese package.

```bash
# En WSL, desde cargo_bot_ws/
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws
colcon build --packages-select cargo_bot_description cargo_bot_navigation cargo_bot_bringup
source install/setup.bash
```

#### Desglose colcon

| Token | Qué hace |
|-------|----------|
| `colcon build` | Builder oficial de ROS 2 |
| `--packages-select cargo_bot_description cargo_bot_navigation cargo_bot_bringup` | Limita el build a estos tres packages. Sin esto, rebuild todo el workspace |

```bash
# Verificar instalación
ls /mnt/c/Users/agusp/Documentos/cargo_bot_ws/install/cargo_bot_navigation/share/cargo_bot_navigation/config/
# Esperar ver: ekf.yaml

ls /mnt/c/Users/agusp/Documentos/cargo_bot_ws/install/cargo_bot_bringup/share/cargo_bot_bringup/launch/
# Esperar ver: localization.launch.py
# Si esta lista está vacía, el setup.py no fue editado bien (revisar arriba)

# Sanity-check del launch ANTES de levantarlo: muestra los args declarados
# sin ejecutar nodos. Si tira traceback Python, hay un bug en el .py.
ros2 launch cargo_bot_bringup localization.launch.py --show-args

# Launch real
ros2 launch cargo_bot_bringup localization.launch.py
```

---

## 7. Verificar IMU+EKF antes de SLAM

### Qué estamos verificando

Antes de meter slam_toolbox encima, queremos confirmar que:
1. `/imu/data` publica correctamente
2. `/odometry/filtered` publica correctamente
3. El TF `odom → base_footprint` viene del EKF (no de Isaac)
4. El subtree URDF `base_footprint → base_link → {wheels, lidar, imu}` viene de `robot_state_publisher` (no de Isaac)
5. Cuando el robot rota, el yaw del filtered tracking es mejor que el del raw odom

### Test 1: topics publicando

```bash
# En tres terminales WSL distintas:
ros2 topic hz /odom                 # ~50 Hz
ros2 topic hz /imu/data             # ~100 Hz
ros2 topic hz /odometry/filtered    # ~30 Hz
```

Los tres deben mostrar las rates indicadas. Si `/odometry/filtered` está silencioso, el EKF no recibe inputs — leé Troubleshooting.

### Test 2: TF tree limpio

```bash
ros2 run tf2_tools view_frames
```

Esto genera un PDF en el directorio actual. Abrilo:

```bash
xdg-open frames.pdf   # o el viewer que tengas
```

Lo que tiene que mostrar el árbol:

| Transform | Broadcaster esperado |
|-----------|----------------------|
| `odom → base_footprint` | `ekf_filter_node` |
| `base_footprint → base_link` | `robot_state_publisher` |
| `base_link → left_wheel_link` | `robot_state_publisher` |
| `base_link → right_wheel_link` | `robot_state_publisher` |
| `base_link → caster_wheel_link` | `robot_state_publisher` |
| `base_link → lidar_link` | `robot_state_publisher` |
| `base_link → imu_link` | `robot_state_publisher` |

Si **cualquiera** dice `broadcaster: /World/cargo_bot/...` o `default_authority` (Isaac), no se borraron bien los TF graphs de Isaac en §4 — re-correr `maintenance/remove_ros_tf_graph.py` y guardar `scene_v4` de nuevo.

Si **cualquiera** dice `Failure` / "no transform from X to Y", `robot_state_publisher` no levantó. Mirar los logs del launch: buscar `[robot_state_publisher-1] [INFO] ... got segment ...` (debe listar los 6 segments). Si no aparecen, falló cargar el xacro — chequear que `cargo_bot_description` fue rebuildeado y que `xacro` está instalado (`which xacro` debe devolver `/opt/ros/humble/bin/xacro`).

### Test 3: Rotación cualitativa

```bash
# Mandar comando de rotación
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}' -r 10
```

En Isaac, robot rota sobre su eje. En paralelo:

```bash
# Comparar las dos fuentes de yaw
ros2 topic echo /odom --field pose.pose.orientation
ros2 topic echo /odometry/filtered --field pose.pose.orientation
```

Si dejás rotar 30 segundos y parás:
- `/odom` yaw va a tener algo de drift acumulado (depende del modelo de wheel slip en Isaac)
- `/odometry/filtered` yaw está corregido por el IMU, más cercano a la pose visual de Isaac

(Si los dos están idénticos, posiblemente el EKF no está fusionando el IMU — chequear que `/imu/data` esté llegando con `ros2 topic echo /imu/data --once`).

### Test 4: Patinaje

Para este test necesitás meter al robot contra una pared en Isaac (puede ser moviendo al robot o creando un cubo justo delante de él) y mandarle `linear.x = 0.5`.

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}' -r 10
```

- `/odom.twist.linear.x` va a reportar ~0.5 m/s (wheel encoders giran pero rueda patina)
- `/odometry/filtered.twist.linear.x` va a reportar valores más bajos porque el IMU accelerometer no detecta aceleración linear → EKF corrige hacia abajo

Esto es **la prueba pragmática** de que la fusion está funcionando.

---

## 8. Configurar slam_toolbox

### Qué estamos haciendo y por qué

`slam_toolbox` es el algoritmo de SLAM 2D más usado en ROS 2. Implementa scan matching + pose graph optimization + loop closure. Versión `async` (la que usamos) corre el processing en thread separado del scan reception, lo que reduce latencia comparado con `sync`.

> **⚠️ GOTCHA RESUELTO 2026-05-30 — scan count mismatch.** El `/scan` de Isaac
> tiene 1066 ranges pero metadata de 360° completo (`angle_min=-pi..angle_max=+pi`),
> y Karto espera `round((max-min)/inc)+1 = 1067` → **rechaza TODOS los scans**
> con `LaserRangeScan contains 1066 range readings, expected 1067` → nunca
> construye el mapa. Fix: un nodo republisher **`scan_angle_fixer`** (en
> `cargo_bot_bringup`) que corrige `angle_max = angle_min + (N-1)*inc` y republica
> `/scan` → `/scan_fixed`. Por eso `scan_topic` abajo es **`/scan_fixed`**, NO
> `/scan`, y `slam.launch.py` arranca el nodo `scan_angle_fixer`. Detalle en
> memoria `cargo_bot_slam_scan_count_mismatch`.

> **⚠️ GOTCHA ESCENA — el lidar necesita PAREDES.** Si el `/scan` se ve como un
> "cono" / cobertura parcial / mapa que salta, verificá PRIMERO que la escena
> Isaac tenga **paredes alrededor del robot**. Sin geometría que golpear, el
> lidar no da returns y slam no puede matchear. NO es un bug del sensor ni hay
> que rotar el lidar (la orientación correcta es IDENTIDAD). Ver memoria
> `cargo_bot_lidar_scan_plane_floor`.

Inputs:
- `/scan_fixed` (del nodo `scan_angle_fixer`, que corrige el `/scan` ~16 Hz de Isaac)
- `/odometry/filtered` (del EKF en step 6)
- TF `odom → base_footprint` (del EKF)
- TF `base_footprint → base_link → lidar_link` (de `robot_state_publisher`, levantado por `localization.launch.py` en §6)

Outputs:
- `/map` (OccupancyGrid)
- TF `map → odom`
- Persistencia: archivos `.posegraph` + `.data` que se pueden cargar después

### Crear `cargo_bot_navigation/config/slam_toolbox.yaml`

```yaml
slam_toolbox:
  ros__parameters:

    # ─── Solver ──────────────────────────────────────────────────────────────
    solver_plugin: solver_plugins::CeresSolver   # Default, robusto
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ─── Frames ──────────────────────────────────────────────────────────────
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan_fixed   # NO /scan — ver gotcha scan count mismatch arriba
    use_sim_time: true     # respetar /clock

    # ─── Mode ────────────────────────────────────────────────────────────────
    mode: mapping          # mapping=construir mapa nuevo. Otras opciones: localization

    # ─── Lifelong mapping (continuar agregando datos al mapa) ────────────────
    enable_interactive_mode: true

    # ─── Tuning del lidar ────────────────────────────────────────────────────
    # IMPORTANTE: ajustar al rango UTIL del lidar real.
    # Nuestro RPLidar S2E sim publica range_max=200m, pero los rays muy lejanos
    # son ruidosos y degradan el mapa. Cap conservador.
    max_laser_range: 12.0

    # ─── Scan matcher ────────────────────────────────────────────────────────
    minimum_time_interval: 0.5   # segundos entre updates del mapa
    transform_publish_period: 0.05   # cada cuánto publica TF map→odom (20 Hz)
    map_update_interval: 5.0     # cada cuánto refresca /map publication (Hz=0.2)
    resolution: 0.05             # metros por pixel del mapa (5 cm)
    minimum_travel_distance: 0.5 # metros entre scans aceptados (más=menos data)
    minimum_travel_heading: 0.5  # radianes entre scans aceptados
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0

    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # ─── Loop closure ────────────────────────────────────────────────────────
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # ─── Correlation parameters ──────────────────────────────────────────────
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # ─── Loop closure correlation ────────────────────────────────────────────
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # ─── Otros ───────────────────────────────────────────────────────────────
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### Cómo funciona slam_toolbox internamente (concepto)

Antes de explicar los parámetros, vale la pena saber qué hace cada parte:

1. **Scan matcher** — cuando llega un nuevo `/scan`, compara con scans anteriores cerca de la pose actual (predicha por `/odometry/filtered`). Busca la rotación + translación que mejor alinea ambos scans. Esto corrige errores de odometría con la geometría del entorno
2. **Pose graph** — cada scan aceptado se convierte en un "nodo" del grafo. Las relaciones entre nodos (transformaciones medidas por el scan matcher) son "aristas"
3. **Loop closure** — cuando el robot vuelve a un lugar ya mapeado, slam_toolbox detecta similitud entre scans separados temporalmente. Agrega una arista al grafo que dice "estos dos nodos deberían estar en la misma pose"
4. **Pose graph optimization** (Ceres) — resuelve un sistema de mínimos cuadrados sobre todas las aristas del grafo para encontrar el set de poses globalmente consistente
5. **Map renderer** — los scans alineados se proyectan al frame `map` y se acumulan en un `nav_msgs/OccupancyGrid` que es lo que publica en `/map`

### Desglose COMPLETO de todos los parámetros

#### Solver (Ceres)

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `solver_plugin` | `solver_plugins::CeresSolver` | Backend de optimización. Ceres es el solver de Google, muy robusto. Alternativas: G2O (más rápido pero menos estable) |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | Algoritmo para resolver el sistema lineal en cada iteración. Sparse Cholesky aprovecha que la mayoría de las relaciones del grafo son entre poses cercanas → matriz sparse |
| `ceres_preconditioner` | `SCHUR_JACOBI` | Acelera convergencia del solver. JACOBI es default robusto |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | Cómo decide los step sizes durante optimización. LM es estándar para mínimos cuadrados no lineales |
| `ceres_dogleg_type` | `TRADITIONAL_DOGLEG` | Variante del trust strategy. No tocar a menos que tengas problemas de convergencia |
| `ceres_loss_function` | None | Función de pérdida. None = quadratic, robusto en sim. Para datos ruidosos del mundo real, considerar `Huber` |

#### Frames y comunicación

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `odom_frame` | `odom` | Frame de origen de odometría. Coincide con el output del EKF |
| `map_frame` | `map` | Frame del mapa global. slam_toolbox publica `map → odom` TF |
| `base_frame` | `base_footprint` | Frame del robot. Coincide con `base_link_frame` del EKF |
| `scan_topic` | `/scan_fixed` | Topic de input. El nodo `scan_angle_fixer` republica acá el `/scan` de Isaac (~16 Hz) con el `angle_max` corregido. NO usar `/scan` directo (lo rechaza Karto) |
| `use_sim_time` | true | Respeta `/clock` |
| `mode` | `mapping` | Modo construcción. Alternativa: `localization` (carga mapa existente y solo localiza) |

#### Tuning del scan matcher

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `max_laser_range` | 12.0 | Cap del rango del lidar en metros. Rays más lejanos se descartan. El RPLidar A2/S2E real tiene rango útil ~12m — capamos al rango real |
| `minimum_time_interval` | 0.5 | Segundos mínimos entre updates del mapa. Procesa scans cada 0.5s mínimo, reduce CPU |
| `transform_publish_period` | 0.05 | Cada cuánto publica TF `map → odom` (1/0.05 = 20 Hz). Más alto = más overhead, menos = TF stale |
| `map_update_interval` | 5.0 | Cada cuántos segundos refresca `/map`. 5s significa que RViz ve update del mapa cada 5s. CPU vs. visualización en tiempo real |
| `resolution` | 0.05 | Metros por pixel. 5cm es estándar Nav2 indoor. Más chico (ej. 0.02) = mapa muy detallado pero pesado. Más grande (ej. 0.10) = mapa rough pero liviano |
| `minimum_travel_distance` | 0.5 | Metros que el robot debe haberse movido entre scans aceptados. <0.5m de movimiento = scan ignorado. Evita procesar scans casi-idénticos |
| `minimum_travel_heading` | 0.5 | Radianes que el robot debe haber rotado entre scans aceptados |
| `scan_buffer_size` | 10 | Cuántos scans recientes mantener en RAM para scan matching local |
| `scan_buffer_maximum_scan_distance` | 10.0 | Metros máximos que se buscan en el buffer para matchear contra el current scan |
| `link_match_minimum_response_fine` | 0.1 | Threshold mínimo de "match quality" en fine search. 0=cualquier match acepta, 1=solo matches perfectos. 0.1 es balanceado |
| `link_scan_maximum_distance` | 1.5 | Metros máximos entre dos scans para considerar matching local |

#### Loop closure

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `do_loop_closing` | true | Activa detección de loop closures. CRÍTICO para mapas grandes |
| `loop_search_maximum_distance` | 3.0 | Metros máximos que se busca un potential loop closure desde la pose actual |
| `loop_match_minimum_chain_size` | 10 | Cuántos scans consecutivos del pasado tiene que mirar para considerar loop closure. Evita falsos positivos por scans aislados |
| `loop_match_maximum_variance_coarse` | 3.0 | Threshold de varianza para fase "coarse" del loop closure. Más alto = más permisivo |
| `loop_match_minimum_response_coarse` | 0.35 | Mínimo response en coarse search para aceptar loop |
| `loop_match_minimum_response_fine` | 0.45 | Mínimo response en fine search (post-coarse) |

#### Correlation parameters (cómo el scan matcher busca alineación)

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `correlation_search_space_dimension` | 0.5 | Metros: el espacio de búsqueda inicial cuando intenta alinear dos scans |
| `correlation_search_space_resolution` | 0.01 | Metros: resolución de la grid de búsqueda. 1cm = búsqueda fina |
| `correlation_search_space_smear_deviation` | 0.1 | Cuánto "smear" se aplica a los scans antes de matching. Suaviza inexactitudes |

#### Loop closure correlation (la búsqueda es más grande porque puede ser un loop lejos)

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `loop_search_space_dimension` | 8.0 | Metros: espacio de búsqueda en loop closure. Más grande que correlation normal porque el loop puede estar a metros de distancia de la pose predicha |
| `loop_search_space_resolution` | 0.05 | Metros: resolución de la grid (no necesita ser tan fina como correlation normal porque busca features grandes) |
| `loop_search_space_smear_deviation` | 0.03 | Smear más bajo porque los loop closures son entre escenas grandes |

#### Otros parámetros de tuning

| Parámetro | Valor | Qué hace |
|-----------|-------|---------|
| `distance_variance_penalty` | 0.5 | Penalización en el optimizer por traslaciones grandes entre scans. Alto = "no muevas la pose tanto" |
| `angle_variance_penalty` | 1.0 | Penalización por rotaciones grandes |
| `fine_search_angle_offset` | 0.00349 | Radianes: step angular fino. Equivale a 0.2° |
| `coarse_search_angle_offset` | 0.349 | Radianes: step angular grueso. Equivale a 20° |
| `coarse_angle_resolution` | 0.0349 | Radianes: resolución angular en coarse search. Equivale a 2° |
| `minimum_angle_penalty` | 0.9 | Penalización mínima angular |
| `minimum_distance_penalty` | 0.5 | Penalización mínima distancia |
| `use_response_expansion` | true | Expande la búsqueda si el primer match es marginal. Mejora detección a costa de CPU |

### Crear `cargo_bot_bringup/launch/slam.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_navigation'),
        'config',
        'slam_toolbox.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('cargo_bot_description'),
        'rviz',
        'cargo_bot.rviz'
    ])

    # First, bring up the EKF localization (input to SLAM)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cargo_bot_bringup'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Isaac simulation clock'
        ),

        localization_launch,

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
```

### Desglose del slam.launch.py

#### `IncludeLaunchDescription` — composición de launches

```python
localization_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('cargo_bot_bringup'),
            'launch',
            'localization.launch.py'
        ])
    ]),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)
```

| Pieza | Qué hace |
|-------|----------|
| `IncludeLaunchDescription` | Acción que **anida** otro launch file. Equivalente a "incluir" un sub-launch como parte del current launch |
| `PythonLaunchDescriptionSource([path])` | Wrapper que dice "el launch file a incluir es un Python launch file en `path`" |
| `PathJoinSubstitution([...])` | Construye el path absoluto al `localization.launch.py` instalado |
| `launch_arguments={'use_sim_time': use_sim_time}.items()` | Pasa argumentos al sub-launch. Es como hacer `ros2 launch ... localization.launch.py use_sim_time:=true` desde la CLI |

**Por qué anidar:** SLAM necesita `odom → base_footprint` TF para arrancar. Esa TF viene del EKF. Si SLAM arrancara antes que el EKF, se quejaría de "no transform from odom to base_footprint" y se quedaría esperando. Anidando `localization.launch.py` adentro del `slam.launch.py`, garantizamos que arrancan en el mismo `ros2 launch` y los nodos descubren la TF en orden.

#### El nodo de slam_toolbox

```python
Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[slam_config, {'use_sim_time': use_sim_time}],
)
```

| Pieza | Qué hace |
|-------|----------|
| `package='slam_toolbox'` | Nombre del package que ya está instalado vía `sudo apt install ros-humble-slam-toolbox` |
| `executable='async_slam_toolbox_node'` | El ejecutable async. Alternativas: `sync_slam_toolbox_node` (procesa scan-by-scan en el thread principal, más simple pero peor latencia), `lifelong_slam_toolbox_node` (continúa mapeando indefinidamente, útil para deployment), `localization_slam_toolbox_node` (carga mapa pre-existente) |
| `name='slam_toolbox'` | **COINCIDE EXACTO con el primer key del YAML** (`slam_toolbox:` línea 1 de `slam_toolbox.yaml`). Si pongo otro nombre acá, los params no se aplican |
| `parameters=[slam_config, {'use_sim_time': use_sim_time}]` | YAML + dict. Override use_sim_time desde la CLI |

#### El nodo de RViz

```python
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen',
)
```

| Pieza | Qué hace |
|-------|----------|
| `arguments=['-d', rviz_config]` | El flag `-d` de RViz le dice "cargá la config desde este archivo". `rviz_config` se resolvió en runtime a `cargo_bot_description/rviz/cargo_bot.rviz` |
| `parameters=[{'use_sim_time': use_sim_time}]` | RViz necesita saber que use sim time para mostrar timestamps correctos |
| (no `name=`) | Si no especificás `name`, RViz usa un nombre auto-generado. No es problema porque RViz no tiene params en YAML que vincular |

#### Por qué este orden en la lista de LaunchDescription

```python
return LaunchDescription([
    DeclareLaunchArgument(...),    # 1. Declarar args primero
    localization_launch,           # 2. Sub-launch del EKF (anidado)
    Node(slam_toolbox),            # 3. SLAM (depende del EKF)
    Node(rviz2),                   # 4. RViz para visualizar
])
```

ROS 2 NO garantiza orden estricto de inicialización entre acciones de un LaunchDescription — todas arrancan más o menos en paralelo. Pero la lista tiene una intención de orden visual que facilita lectura. La dependencia real (SLAM espera al EKF) se resuelve vía TF: SLAM bloquea hasta ver la TF que el EKF publica.

### Build + verificar

```bash
cd /mnt/c/Users/agusp/Documentos/cargo_bot_ws
colcon build --packages-select cargo_bot_navigation cargo_bot_bringup
source install/setup.bash
```

---

## 9. Mapear el ambiente

### Qué estamos haciendo y por qué

Con todo el stack arriba, el robot ahora puede generar un mapa. Lo movemos por la escena manualmente (teleop), slam_toolbox va construyendo el mapa, y RViz lo muestra en vivo.

### Comandos (en terminales WSL separadas)

#### Terminal 1: Boot completo

Asegurate primero que esté:
- Discovery server corriendo
- Isaac Sim con scene_v4 en ▶ Play

Después:

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/source_ros_wsl.sh
source /mnt/c/Users/agusp/Documentos/cargo_bot_ws/install/setup.bash

ros2 launch cargo_bot_bringup slam.launch.py
```

Output esperado: log con `ekf_filter_node` arrancando, después `slam_toolbox` (con varias líneas de Ceres init), y RViz abriéndose.

#### Terminal 2: Teleop para mover el robot

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/agusp/Documentos/cargo_bot_ws/config/source_ros_wsl.sh

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teclas (estándar):
- `i` = forward
- `,` = backward
- `j` / `l` = rotate left/right
- `space` = stop

### Estrategia de mapeo

1. **Rotar 360°** en el spot inicial — slam_toolbox necesita ver el entorno completo desde un punto para inicializar.
2. **Mover en línea recta despacio** hacia adelante 2-3 metros.
3. **Rotar 360° de nuevo** en el nuevo spot.
4. Repetir avanzando + rotando hasta cubrir todo el ambiente.
5. **Volver al spot inicial** para que se cierre el loop. RViz va a mostrar un "snap" cuando detecta el loop closure — el mapa se corrige.
6. **Mirar el mapa en RViz** mientras vas — si ves desalineamiento o paredes "torcidas", indica que la odometría está mal o el lidar publica mal.

### Tips de mapeo

- **Velocidad lenta** (linear.x ≤ 0.3 m/s, angular.z ≤ 0.5 rad/s). Más rápido = más ruido en el scan matching
- **No teleportar el robot** en Isaac (mover via cmd_vel siempre). Si lo "teleportás" agarrando el prim con el mouse, slam_toolbox se va a confundir
- **Si hay ventanas o espejos** en la escena: el lidar va a "ver" lo que hay del otro lado → mapa raro. Para Fase 3 inicial usar escena con paredes opacas

---

## 10. Guardar el mapa

### Qué estamos haciendo y por qué

Una vez que el mapa visual en RViz se ve bien, queremos persistirlo a disco para que Fase 4 (Nav2) pueda cargarlo después con AMCL en modo localización.

slam_toolbox tiene dos servicios para guardar:

- `/slam_toolbox/save_map` → guarda un `.pgm` + `.yaml` legible por Nav2
- `/slam_toolbox/serialize_map` → guarda el pose graph completo (`.posegraph` + `.data`) — útil para continuar mapeando después

### Comandos

```bash
# Crear carpeta para el mapa (si no existe)
mkdir -p /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_navigation/maps

# Guardar mapa estándar (Nav2 format)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_navigation/maps/scene_v4'}}"
```

#### Desglose

| Token | Qué hace |
|-------|----------|
| `ros2 service call` | Llama un servicio sincrónicamente |
| `/slam_toolbox/save_map` | El nombre del servicio. Sin namespace porque slam_toolbox vive en el root |
| `slam_toolbox/srv/SaveMap` | Tipo del servicio |
| `"{name: {data: '...'}}"` | Argumento. El servicio espera un `std_msgs/String` con el nombre/path del archivo (sin extensión, agrega `.pgm` y `.yaml` solo) |

Después del comando se crean:
- `cargo_bot_navigation/maps/scene_v4.pgm` — imagen del mapa (P5 grayscale)
- `cargo_bot_navigation/maps/scene_v4.yaml` — metadata (resolución, origin, thresholds)

### Verificación

```bash
ls -la /mnt/c/Users/agusp/Documentos/cargo_bot_ws/src/cargo_bot_navigation/maps/
```

Esperar ver los dos archivos. El `.pgm` lo podés abrir con cualquier viewer de imágenes (Windows Photos, IrfanView, etc.) — vas a ver tu mapa: negro=ocupado, blanco=libre, gris=desconocido.

---

## 11. Verificación final end-to-end

Checklist completo para cerrar Fase 3:

```
[ ] /clock publica ~50 Hz desde Isaac scene_v4
[ ] /odom publica ~50 Hz desde Isaac
[ ] /imu/data publica ~100 Hz desde Isaac
[ ] /scan publica ~16 Hz desde Isaac
[ ] /odometry/filtered publica ~30 Hz desde EKF
[ ] TF tree limpio: odom→base_footprint viene del ekf_filter_node, no de Isaac
[ ] Robot navega con teleop sin que el mapa se desalinea visualmente
[ ] Mapa guardado en cargo_bot_navigation/maps/scene_v4.pgm + .yaml
[ ] El .pgm abierto en un image viewer coincide con la escena de Isaac
[ ] Loop closure detectado al menos una vez (visible en log de slam_toolbox)
```

Done. Listo para Fase 4a (Nav2 core).

---

## 12. Troubleshooting

### Síntoma: `/imu/data` no publica

**Causa probable 1:** el URDF Importer no creó el sensor IMU prim bajo `imu_link`, y el IsaacReadIMU del OmniGraph tiene `imuPrim` apuntando a algo que no existe.

**Fix:**
1. En Isaac Stage panel, expandir `/World/cargo_bot/imu_link`. ¿Hay algún sub-prim?
2. Si no: agregarlo manual via **Create → Sensors → IMU Sensor** apuntando a `/World/cargo_bot/imu_link`.
3. Ajustar `IsaacReadIMU.imuPrim` para apuntar al nuevo prim.

**Causa probable 2:** el OmniGraph del IMU no está siendo ticked.

**Fix:** verificar que `OnPlaybackTick` esté conectado a `IsaacReadIMU.execIn`.

### Síntoma: `/odometry/filtered` no publica

**Causa probable 1:** el EKF no recibe inputs porque `use_sim_time` está mal.

**Fix:** ver el output del launch — `ekf_filter_node` debería loguear `Setting parameter 'use_sim_time' = true`. Si dice false o no aparece, revisar el launch file.

**Causa probable 2:** los topics `/odom` y `/imu/data` están publicando con QoS incompatibles con lo que espera el EKF.

**Fix:**
```bash
ros2 topic info /odom -v
ros2 topic info /imu/data -v
```

Si dice `Reliability: BEST_EFFORT`, el EKF (que es RELIABLE por default) no los va a leer. Cambiar en el Property panel del Isaac OmniGraph → set QoS a RELIABLE.

### Síntoma: TF tree con `odom→base_footprint` duplicado

**Causa:** no desconectaste bien la rama RawTransformTree del Odometry OmniGraph en Isaac (step 4.4).

**Fix:** volver al OmniGraph de Odometry en scene_v4. Verificar visualmente que el output de `IsaacComputeOdometry` NO está conectado a ningún `RawTransformTree` ni `ROS2PublishRawTransformTree`.

### Síntoma: mapa se desalinea cuando el robot rota

**Causa:** odometría inestable. Posibles:
- El EKF no está fusionando bien el IMU
- Los timestamps no están alineando

**Fix:**
1. `ros2 topic echo /imu/data --once` para confirmar que IMU publica datos válidos (no NaNs, no todo cero).
2. `ros2 topic echo /odom --once` para confirmar wheel odom.
3. Si ambos OK, posiblemente el problema es covariance — bajar `process_noise_covariance` del EKF.

### Síntoma: slam_toolbox lagea o consume mucha CPU

**Fix:**
- Subir `minimum_travel_distance` y `minimum_travel_heading` (procesa menos scans)
- Bajar `max_laser_range` aún más (procesa menos rays por scan)
- Subir `resolution` a 0.10 (mapa más bajo-res pero más rápido)

---

## 13. Tips de rendimiento

- **GPU 4060 Laptop 8GB:** Isaac suele estar al límite. Cerrar otros browsers / Discord / Streamlabs antes de mapear sesiones largas.
- **Velocidad de mapeo:** mover el robot DESPACIO (linear.x ≤ 0.3 m/s) y rotar despacio (angular.z ≤ 0.5 rad/s). Más rápido = más ruido = peor mapa.
- **Tamaño del mapa:** si mapas grandes (>30m × 30m), considerar dividir en sub-mapas. Para casa o oficina chica un solo mapa anda perfecto.
- **CPU WSL:** slam_toolbox + EKF + RViz juntos pueden saturar 4 threads de WSL. Si lagea, cerrar RViz y mirar el mapa con `ros2 run nav2_map_server map_saver_cli --occ 70 --free 30 -f /tmp/snapshot` cada N segundos.
- **Para iterar rápido:** se puede deshabilitar `do_loop_closing` durante test inicial de geometría, después activarlo para el mapeo "final" que vas a guardar.
