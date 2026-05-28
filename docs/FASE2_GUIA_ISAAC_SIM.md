# Fase 2 — Isaac Sim: Importar, Configurar y Controlar el Robot

> **Objetivo:** El robot se mueve en Isaac Sim controlado desde WSL2.
> **Verificacion final:** `teleop_twist_keyboard` en WSL2 mueve el robot en Isaac Sim,
> y `ros2 topic echo` muestra datos en `/odom` y `/scan`.

---

## Indice

1. [Preparar el entorno](#1-preparar-el-entorno)
2. [Importar URDF a Isaac Sim](#2-importar-urdf-a-isaac-sim)
3. [Configurar la fisica del robot](#3-configurar-la-fisica-del-robot)
4. [Agregar el LiDAR RTX](#4-agregar-el-lidar-rtx)
5. [Crear el Action Graph (OmniGraph)](#5-crear-el-action-graph-omnigraph)
6. [Crear la escena](#6-crear-la-escena)
7. [Verificacion final](#7-verificacion-final)
8. [Troubleshooting](#8-troubleshooting)
9. [Tips de rendimiento](#9-tips-de-rendimiento)

---

## 1. Preparar el entorno

Antes de abrir Isaac Sim, asegurate de que el DDS esté funcionando.

### Checklist pre-inicio

```
[ ] Discovery Server corriendo en WSL2
      → En una terminal WSL: source ~/cargo_bot_ws/config/source_ros_wsl.sh
      → Luego: ~/cargo_bot_ws/config/start_discovery_server.sh

[ ] Isaac Sim lanzado con las variables DDS
      → Doble click en: cargo_bot_ws\config\launch_isaac_ros.cmd

[ ] Verificar conexion
      → En WSL: ros2 topic list
      → Deberias ver al menos /clock (si ya tenes un Action Graph con Publish Clock)
```

> **Nota:** Si no ves `/clock`, revisá la Fase 0 o preguntame.

---

## 2. Importar URDF a Isaac Sim

### 2.1 Generar el URDF procesado

#### Que estamos haciendo y por que

Tus archivos `.xacro` son templates — tienen variables (`${wheel_radius}`), macros
(`xacro:inertial_box`), e includes (`xacro:include`). Isaac Sim no entiende xacro,
solo URDF puro (XML plano). Entonces el flujo es:

1. **`colcon build`** — copia los archivos del paquete (xacros, meshes, launch) a la
   carpeta `install/` donde ROS 2 los puede encontrar con `package://`
2. **`xacro ... > cargo_bot.urdf`** — procesa todos tus xacros: resuelve variables,
   expande macros, fusiona includes, y genera un unico archivo `.urdf` con XML puro
3. **`check_urdf`** — valida que el URDF sea correcto: links conectados, joints bien
   definidos, sin loops. Te muestra el tree si esta bien
4. **Copiar a Windows** — Isaac Sim corre en Windows nativo y no puede leer paths
   internos de WSL. Por eso copias el `.urdf` a `/mnt/c/...` (el disco C visto desde WSL)

Despues en Isaac Sim el URDF Importer lee ese archivo y lo convierte a **USD**
(el formato nativo de Isaac/Omniverse), creando los prims con fisica, joints y meshes.

#### Comandos

Primero hay que compilar el paquete y generar el `.urdf` final desde los `.xacro`:

```bash
# En WSL2
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/agusp/cargo_bot_ws
colcon build --packages-select cargo_bot_description
source install/setup.bash

# Generar el URDF final (para ROS 2 / check_urdf)
xacro src/cargo_bot_description/urdf/cargo_bot.urdf.xacro > /tmp/cargo_bot.urdf

# Verificar que no tenga errores
check_urdf /tmp/cargo_bot.urdf
# Si no esta instalado: sudo apt install liburdfdom-tools

# Generar version para Isaac Sim (paths absolutos de Windows)
# Isaac Sim NO entiende package:// asi que reemplazamos los paths
xacro src/cargo_bot_description/urdf/cargo_bot.urdf.xacro \
  | sed 's|package://cargo_bot_description/|C:/Users/agusp/cargo_bot_ws/src/cargo_bot_description/|g' \
  > /mnt/c/Users/agusp/cargo_bot_ws/cargo_bot_isaac.urdf
```

> **Importante:** Los xacro originales usan `package://` y eso es CORRECTO para ROS 2
> (RViz, Nav2, SLAM). NO los cambies. Solo se genera `cargo_bot_isaac.urdf` con paths
> absolutos para Isaac Sim. Cada vez que modifiques los xacro y quieras re-importar
> en Isaac, volve a correr el comando de arriba.

### 2.2 Verificar extension URDF Importer

Antes de importar, confirmar que la extension este habilitada:

1. **Window → Extensions**
2. Buscar `isaacsim.asset.importer.urdf`
3. Verificar que el toggle este activado (normalmente viene ON por defecto)

### 2.3 Importar en Isaac Sim

1. En Isaac Sim: **File → Import**
2. Seleccionar `C:\Users\agusp\cargo_bot_ws\cargo_bot.urdf`
3. Se abre un panel con las opciones de importacion. Configurar:

| Opcion | Valor | Por que |
|--------|-------|---------|
| **Base Type** | `Moveable base` | Es un robot movil, no un brazo fijo |
| **Drive Type** | `Force` | Control por fuerza/torque (standard para diferencial) |
| **Drive Target** | `Velocity` | Control diferencial usa velocidades |
| **Collision** | `Convex Hull` | Genera colisiones a partir de los meshes |
| **Self Collision** | `OFF` | No necesario para robot simple |
| **Default Density** | default | Usa las masas/inercias de tu URDF |

4. Click **Import**
5. El robot aparece en el Stage como un prim USD

### 2.3 Verificar la importacion

Despues de importar, checkeá en el panel **Stage** (lado derecho):

```
/World
  └── cargo_bot
        ├── base_footprint          (Xform)
        │     └── base_link         (Xform + mesh visual/collision)
        │           ├── left_wheel_link    (Xform + mesh)
        │           ├── right_wheel_link   (Xform + mesh)
        │           ├── caster_wheel_link  (Xform + mesh)
        │           ├── lidar_link         (Xform + mesh)
        │           └── imu_link           (Xform)
        └── Joints
              ├── left_wheel_joint     (RevoluteJoint)
              ├── right_wheel_joint    (RevoluteJoint)
              ├── caster_wheel_joint   (FixedJoint)
              ├── lidar_joint          (FixedJoint)
              └── imu_joint            (FixedJoint)
```

> **Si algo falta o esta mal:** NO re-importes encima. Borra el prim `/World/cargo_bot` y volve a importar.

---

## 3. Configurar la fisica del robot

### 3.1 Articulation Root

El importer deberia haberlo puesto automaticamente. Verificar:

1. Seleccionar `/World/cargo_bot/base_footprint` en el Stage
2. En el panel **Property** (abajo a la derecha), buscar **Physics → Articulation Root**
3. Si NO esta: click derecho → **Add → Physics → Articulation Root**

> **Importante:** El Articulation Root debe estar en el prim **raiz** del robot (base_footprint),
> no en base_link ni en un joint.

### 3.2 Wheel Joint Drives

Las ruedas necesitan drives de velocidad para que el Differential Controller las mueva.

Para cada joint de rueda (`left_wheel_joint`, `right_wheel_joint`):

1. Seleccionar el joint en el Stage
2. En **Property → Physics → Drive**, verificar:

| Propiedad | Valor |
|-----------|-------|
| **Type** | `angular` |
| **Target Type** | `velocity` |
| **Damping** | `1e3` |
| **Stiffness** | `0` (para velocity drive, stiffness = 0) |
| **Max Force** | `1e4` |

> Si los drives no existen, agregarlos: click derecho en el joint → **Add → Physics → Angular Drive**

### 3.3 Caster Wheel

El caster es un joint fijo. No necesita drive. Verificar que:
- El joint sea **Fixed**
- La colision del caster tenga **friccion baja**:
  1. Seleccionar el collision mesh del caster en Stage
  2. **Property → Physics → Physics Material**
  3. Crear un nuevo material con: **Static Friction = 0.0**, **Dynamic Friction = 0.0**

### 3.4 Probar la fisica basica

1. Click **Play** (barra superior)
2. El robot deberia caer y apoyarse en el ground plane por gravedad
3. Si flota, atraviesa el suelo o se comporta raro:
   - Verificar que hay un **Ground Plane** (si no: Create → Physics → Ground Plane)
   - Verificar que las collision meshes estan habilitadas
   - Verificar que Articulation Root esta en el prim correcto

---

## 4. Agregar el LiDAR RTX

### 4.1 Crear el sensor

1. Seleccionar `/World/cargo_bot/.../lidar_link` en el Stage
2. **Create → Isaac → Sensors → RTX Lidar**
3. El sensor se agrega como hijo de `lidar_link`

### 4.2 Configurar parametros (emulando RPLidar A1)

Seleccionar el prim del lidar RTX y en **Property**, configurar:

| Parametro | Valor | Notas |
|-----------|-------|-------|
| **Draw Points** | `ON` | Para ver el scan en el viewport |
| **Rotation Rate** | `5.5` Hz | Frecuencia del RPLidar A1 |
| **High LOD** | `OFF` | Ahorra GPU |

Para los parametros especificos del scan, necesitas un **sensor config**. Isaac Sim usa archivos JSON para definir el patron del lidar. Podes:

**Opcion A — Configurar via OmniGraph (mas simple):**
Los parametros del scan se definen en el nodo `ROS2 Publish LaserScan` del Action Graph (seccion 5).

**Opcion B — Config JSON custom:**
Crear un archivo `rplidar_a1_config.json`:

```json
{
    "class": "sensor",
    "type": "lidar",
    "name": "RPLidar_A1",
    "driveWorksId": "GENERIC",
    "profile": {
        "scanType": "rotary",
        "intensityProcessing": "normalization",
        "rayType": "IDEALIZED",
        "nearRangeM": 0.15,
        "farRangeM": 12.0
    },
    "firing": {
        "cycleDurationMs": 181.818,
        "scanFrequencyHz": 5.5,
        "numberOfChannels": 1,
        "emitterStates": [
            {
                "fireTimeNs": 0,
                "scanAzimuthDeg": 0.0,
                "scanElevationDeg": 0.0
            }
        ],
        "scanPattern": "uniform",
        "numScansH": 360,
        "numScansV": 1,
        "scanRangeH": [-180.0, 180.0],
        "scanRangeV": [0.0, 0.0]
    }
}
```

Guardar en: `C:\Users\agusp\cargo_bot_ws\config\rplidar_a1_config.json`
Y referenciarlo en las propiedades del sensor RTX Lidar → **Sensor Config File**.

### 4.3 Verificar el sensor

1. **Play** la simulacion
2. Deberias ver puntos del lidar dibujados en el viewport (si Draw Points esta ON)
3. Si no ves puntos: verificar que el lidar no esta dentro de un mesh (moverlo ligeramente arriba si es necesario)

---

## 5. Crear el Action Graph (OmniGraph)

El Action Graph conecta la simulacion con ROS 2. Es la pieza central de la Fase 2.

### 5.1 Crear el grafo

1. **Window → Visual Scripting → Action Graph**
2. Se abre el editor de OmniGraph
3. Click **New Action Graph**

### 5.2 Agregar nodos

Buscar cada nodo en el panel de busqueda del editor y arrastrarlo al grafo.
Despues conectar las salidas a las entradas como se indica.

#### Nodo 1: Tick + Context

| Nodo | Para que |
|------|----------|
| **On Playback Tick** | Dispara todo cada frame de simulacion |
| **ROS2 Context** | Configura el bridge DDS (usa el Discovery Server que ya tenes) |

> El nodo **ROS2 Context** lo podes dejar con valores default. Usa las env vars
> que seteó `launch_isaac_ros.cmd` (RMW, FASTRTPS profile, DOMAIN_ID).

#### Nodo 2: Recibir comandos de velocidad

| Nodo | Configuracion |
|------|---------------|
| **ROS2 Subscribe Twist** | `topicName`: `/cmd_vel` |

**Conexiones:**
```
On Playback Tick [Tick] ──► ROS2 Subscribe Twist [Exec In]
ROS2 Context [Context] ──► ROS2 Subscribe Twist [Context]
```

#### Nodo 3: Differential Controller

| Nodo | Configuracion |
|------|---------------|
| **Differential Controller** | `wheelDistance`: 0.29 (tu wheel_separation) |
| | `wheelRadius`: 0.1 (tu wheel_radius) |
| | `maxWheelSpeed`: 10.0 (rad/s, ajustar luego) |

**Conexiones:**
```
On Playback Tick [Tick] ──────────────────► Differential Controller [Exec In]
ROS2 Subscribe Twist [Linear Velocity] ──► Differential Controller [Linear Velocity]
ROS2 Subscribe Twist [Angular Velocity] ─► Differential Controller [Angular Velocity]
```

> **Atencion:** El Subscribe Twist da `linear` y `angular` como vectores.
> El Differential Controller espera valores escalares (float).
> Si no conecta directo, necesitas nodos **Break 3-Vector** intermedios
> para extraer el componente `x` de linear y `z` de angular.

#### Nodo 4: Articulation Controller

| Nodo | Configuracion |
|------|---------------|
| **Articulation Controller** | `robotPath`: `/World/cargo_bot` |
| | `jointNames`: `["left_wheel_joint", "right_wheel_joint"]` |
| | `usePath`: `true` |

**Conexiones:**
```
Differential Controller [Exec Out] ──────────► Articulation Controller [Exec In]
Differential Controller [Velocity Command] ──► Articulation Controller [Velocity Command]
```

> **Importante:** el `robotPath` tiene que coincidir EXACTO con el path del prim
> en el Stage. Si tu robot se llama diferente, ajustalo.

#### Nodo 5: Publicar Odometria

| Nodo | Configuracion |
|------|---------------|
| **ROS2 Publish Odometry** | `topicName`: `/odom` |
| | `chassisPrim`: `/World/cargo_bot/base_footprint` |
| | `odomPrim`: `/World` (o un Xform vacio que represente odom frame) |

**Conexiones:**
```
On Playback Tick [Tick] ──► ROS2 Publish Odometry [Exec In]
ROS2 Context [Context] ──► ROS2 Publish Odometry [Context]
```

#### Nodo 6: Publicar LaserScan

| Nodo | Configuracion |
|------|---------------|
| **ROS2 Publish LaserScan** | `topicName`: `/scan` |
| | `frameId`: `lidar_link` |

**Conexiones:**
```
On Playback Tick [Tick] ──► ROS2 Publish LaserScan [Exec In]
ROS2 Context [Context] ──► ROS2 Publish LaserScan [Context]
```

> Conectar la salida del sensor RTX Lidar al input de este nodo.

#### Nodo 7: Publicar TF

| Nodo | Configuracion |
|------|---------------|
| **ROS2 Publish Transform Tree** | `targetPrims`: `/World/cargo_bot` |

**Conexiones:**
```
On Playback Tick [Tick] ──► ROS2 Publish Transform Tree [Exec In]
ROS2 Context [Context] ──► ROS2 Publish Transform Tree [Context]
```

#### Nodo 8: Publicar Clock

Si ya tenes uno del setup de Fase 0, no dupliques. Si no:

| Nodo | Configuracion |
|------|---------------|
| **ROS2 Publish Clock** | (default) |

**Conexiones:**
```
On Playback Tick [Tick] ──► ROS2 Publish Clock [Exec In]
ROS2 Context [Context] ──► ROS2 Publish Clock [Context]
```

### 5.3 Diagrama completo del Action Graph

```
                    ┌──────────────────┐
                    │ On Playback Tick │
                    └────────┬─────────┘
                             │ Tick
          ┌──────────────────┼──────────────────────────────────┐
          │                  │                                  │
          ▼                  ▼                                  ▼
┌─────────────────┐ ┌───────────────┐ ┌──────────────────────────────────┐
│ ROS2 Subscribe  │ │ ROS2 Publish  │ │ ROS2 Publish Clock               │
│ Twist (/cmd_vel)│ │ Odometry      │ │ ROS2 Publish Transform Tree      │
└────────┬────────┘ │ (/odom)       │ │ ROS2 Publish LaserScan (/scan)   │
         │          └───────────────┘ └──────────────────────────────────┘
         │ linear.x, angular.z
         ▼
┌─────────────────────────┐
│ Differential Controller │
│ wheelDist=0.29          │
│ wheelRadius=0.1         │
└────────┬────────────────┘
         │ velocity command [left, right]
         ▼
┌─────────────────────────┐
│ Articulation Controller │
│ robotPath=/World/       │
│           cargo_bot     │
└─────────────────────────┘
```

### 5.4 Guardar la escena

**File → Save As** → `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scenes\cargo_bot_scene.usd`

> Guarda seguido. Isaac Sim puede crashear y perdes el Action Graph.

---

## 6. Crear la escena

### 6.1 Ground Plane

Si no existe: **Create → Physics → Ground Plane**

### 6.2 Habitacion simple

Crear 4 paredes con cubos:

1. **Create → Mesh → Cube**
2. Escalar para hacer una pared (ej: 5m x 0.1m x 2m)
3. Posicionar formando un rectangulo
4. Repetir x4

Ejemplo de layout:

```
         Pared Norte (5m x 0.1m x 2m) @ y = 2.5
    ┌─────────────────────────────────┐
    │                                 │
    │   Pared      Habitacion    Pared│
    │   Oeste                    Este │
    │  (0.1x5x2)               (0.1x5x2)
    │   @ x=-2.5               @ x=2.5│
    │                                 │
    │         ┌───┐                   │
    │         │BOT│  ← tu robot       │
    │         └───┘                   │
    │                                 │
    │     □        □       □          │
    │   cubo1    cubo2   cubo3        │
    │  (obstaculos)                   │
    └─────────────────────────────────┘
         Pared Sur (5m x 0.1m x 2m) @ y = -2.5
```

### 6.3 Obstaculos

Agregar 3-5 cubos de distintos tamanios como obstaculos:
- **Create → Mesh → Cube**
- Escalar a tamanios variados (0.3m, 0.5m, 0.8m)
- Distribuirlos por la habitacion
- Agregarles colision: seleccionar → **Add → Physics → Collision Preset**

### 6.4 Agregar colision a las paredes

Para cada pared:
1. Seleccionar el prim
2. Click derecho → **Add → Physics → Collision Preset**
3. Verificar que tenga **Collider** en Properties

---

## 7. Verificacion final

### 7.1 Preparar WSL2

Abrir **3 terminales** en WSL2. En cada una:

```bash
source /mnt/c/Users/agusp/cargo_bot_ws/config/source_ros_wsl.sh
source /mnt/c/Users/agusp/cargo_bot_ws/install/setup.bash
```

### 7.2 Checklist de verificacion

**Terminal 1 — Ver topics:**
```bash
ros2 topic list
```
Deberian aparecer:
```
/clock
/cmd_vel
/odom
/scan
/tf
/tf_static
```

**Terminal 2 — Teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> Si no esta instalado: `sudo apt install ros-humble-teleop-twist-keyboard`

**Terminal 3 — Verificar datos:**

```bash
# Odometria (mover el robot con teleop y ver que cambia)
ros2 topic echo /odom --once

# LaserScan (ver rangos del lidar)
ros2 topic echo /scan --once

# TF tree completo
ros2 run tf2_tools view_frames
# Genera un PDF con el arbol de transforms
```

### 7.3 Criterios de exito

```
[✓] El robot se mueve con teleop_twist_keyboard
    → adelante/atras/girar responde a las teclas

[✓] /odom muestra pose que cambia al mover
    → position.x, position.y cambian

[✓] /scan muestra rangos validos
    → ranges[] tiene valores entre 0.15 y 12.0

[✓] El TF tree es correcto
    → odom → base_footprint → base_link → {wheels, lidar, imu}

[✓] El robot no atraviesa paredes ni obstaculos

[✓] El robot no flota ni se hunde
```

---

## 8. Troubleshooting

### El robot no se mueve con teleop

| Problema | Solucion |
|----------|----------|
| `/cmd_vel` no llega a Isaac | Verificar `ros2 topic echo /cmd_vel` en WSL. Si se ve pero Isaac no reacciona → revisar DDS/Discovery Server |
| Subscribe Twist no recibe | Verificar que el topicName sea exactamente `/cmd_vel` en el nodo OmniGraph |
| Ruedas no giran | Verificar Joint Drives: Type=angular, Target=velocity, Stiffness=0 |
| Articulation Controller error | Verificar `robotPath` y `jointNames` coinciden con el Stage |
| Se mueve muy lento/rapido | Ajustar `wheelRadius` y `wheelDistance` en Differential Controller |

### No aparecen topics en WSL

| Problema | Solucion |
|----------|----------|
| Discovery Server caido | Relanzar: `~/cargo_bot_ws/config/start_discovery_server.sh` |
| Env vars no seteadas | `echo $ROS_DISCOVERY_SERVER` debe mostrar la IP:11811 |
| Isaac no tiene ROS2 Context | Agregar nodo ROS2 Context en el Action Graph |
| Firewall bloqueando | Verificar regla UDP 7400-7420 + 11811 |

### El LiDAR no publica

| Problema | Solucion |
|----------|----------|
| `/scan` no aparece | Verificar nodo ROS2 Publish LaserScan en el Action Graph |
| Ranges todos `inf` | El lidar esta dentro de un mesh o apuntando al cielo. Verificar posicion y orientacion |
| Ranges todos `0` | Rango minimo muy alto o el sensor esta deshabilitado |
| Pocos puntos | Verificar samples/config del RTX Lidar |

### Errores de fisica

| Problema | Solucion |
|----------|----------|
| Robot cae infinitamente | Falta Ground Plane |
| Robot atraviesa suelo | Collision meshes deshabilitadas |
| Robot vibra/explota | Damping muy bajo o masas/inercias irreales |
| Caster se traba | Verificar friccion 0 en el material del caster |

---

## 9. Tips de rendimiento

Tu GPU es una RTX 4060 Laptop con 8 GB VRAM. Tené en cuenta:

| Ajuste | Recomendacion |
|--------|---------------|
| **Resolucion de render** | 720p o menos |
| **RTX Lidar** | 360 rays max (ya configurado) |
| **Ray-traced reflections** | **OFF** (Render Settings → Ray Tracing) |
| **Ray-traced GI** | **OFF** |
| **Path Tracing** | **OFF** (usar RTX Real-Time) |
| **Robots en escena** | 1 solo |
| **Meshes de colision** | Primitivas (cajas/cilindros) siempre que se pueda |
| **Physics step** | 60 Hz default esta bien |

Si aun asi lagea:
- Bajar render a 480p
- Reducir samples del lidar a 180
- Cerrar otros programas que usen GPU

---

## Links utiles

- [Isaac Sim 5.1 — URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html)
- [Isaac Sim 5.1 — OmniGraph ROS2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2/ros2_omnigraph_nodes.html)
- [Isaac Sim 5.1 — RTX Lidar Sensor](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/rtx_lidar.html)
- [Isaac Sim 5.1 — Differential Drive Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2/ros2_drive_robot.html)
- [ROS 2 teleop_twist_keyboard](https://index.ros.org/p/teleop_twist_keyboard/)

---

## Anexo — Nodos OmniGraph: que son y para que sirven

OmniGraph (OG) es el sistema de **dataflow + execution graph** de Isaac Sim.
Cada nodo es una "caja" que recibe inputs y produce outputs. Los nodos se
conectan con dos tipos de cables:

- **Cables de execucion (exec)** — flechas que indican QUIEN dispara a quien.
  Tipicamente salen de `tick`, `execOut`, `step`. Cuando el dato entra al
  pin `execIn` de un nodo, ese nodo "compute" su logica una vez.
- **Cables de datos** — pasan valores (numeros, vectores, paths, etc.) entre
  outputs y inputs.

En Isaac, los nodos OG corren en cada `simulation step` (cuando esta en Play).

A continuacion explico los nodos que usamos en el cargo_bot, agrupados por
funcion.

---

### A. Trigger + contexto

#### `On Playback Tick`

- **Que es:** El "reloj" del grafo. Tiene un pin `tick` de execution.
- **Que hace:** Dispara una vez por simulation frame (~60 Hz por default).
  Es el origen de TODO el flujo de ejecucion del grafo.
- **Inputs:** ninguno relevante.
- **Outputs:** `tick` (exec, dispara downstream), `deltaSeconds` (double, dt
  del frame).
- **Cuando usarlo:** SIEMPRE como punto de entrada de un grafo que tiene
  que correr cada frame.

#### `ROS2 Context`

- **Que es:** Wrapper de un contexto DDS de ROS 2. Internamente mantiene un
  handle `rclcpp::Context`.
- **Que hace:** Provee el "canal" por el que TODOS los nodos ROS 2 publican
  y suscriben. Se conecta una sola vez y se reusa.
- **Inputs:** `domain_id` (uint64, debe coincidir con `ROS_DOMAIN_ID` de WSL).
- **Outputs:** `context` (uint64 handle).
- **Cuando usarlo:** SIEMPRE, una unica instancia por grafo, conectada a TODOS
  los nodos `ROS2 Subscribe *` y `ROS2 Publish *`.

---

### B. cmd_vel (subscribe + diferencial)

#### `ROS2 Subscribe Twist`

- **Que es:** Suscriptor ROS 2 de mensajes `geometry_msgs/msg/Twist`.
- **Que hace:** Cada vez que recibe un mensaje en el topic configurado,
  lo pone en sus outputs. Si no recibe nada, los outputs quedan en 0.
- **Inputs:** `topicName` (string, ej. `/cmd_vel`), `context` (del nodo
  ROS2 Context), `qosProfile`.
- **Outputs:** `linearVelocity` (vec3d, m/s), `angularVelocity` (vec3d,
  rad/s), `execOut` (exec).

#### `Break 3-Vector`

- **Que es:** Utility node que descompone un vec3 en sus 3 componentes.
- **Que hace:** Recibe un `(x, y, z)` y emite los tres como doubles
  separados.
- **Inputs:** `tuple` (vec3d).
- **Outputs:** `x`, `y`, `z` (double cada uno).
- **Por que lo usamos:** El Subscribe Twist da `linear` y `angular` como
  vec3, pero el Differential Controller espera `linearVelocity` y
  `angularVelocity` como **escalares double**. Necesitamos dos Break Vector:
  uno para sacar el `x` del lineal (avance), otro para sacar el `z` del
  angular (giro yaw).

#### `Differential Controller`

- **Que es:** Cinematica inversa de diff-drive: convierte velocidad lineal
  + angular del robot a velocidad angular de cada rueda.
- **Que hace:** Usa la formula clasica:
  - `v_left  = (linear - angular * wheel_distance / 2) / wheel_radius`
  - `v_right = (linear + angular * wheel_distance / 2) / wheel_radius`
- **Inputs:**
  - `wheelDistance` (double, separacion entre ruedas en metros)
  - `wheelRadius` (double, radio de rueda)
  - `linearVelocity` (m/s), `angularVelocity` (rad/s)
  - `maxLinearSpeed`, `maxAngularSpeed`, `maxWheelSpeed` (limites de
    seguridad — IMPORTANTE setearlos, sino 0 = ilimitado)
- **Outputs:** `velocityCommand` (double[2], orden `[left, right]`).

#### `Articulation Controller`

- **Que es:** Puente entre OmniGraph y PhysX. Aplica comandos a joints de
  un articulation root.
- **Que hace:** Recibe un array de velocidades (o posiciones) y un array
  de joint names, y los envia al PhysX articulation solver.
- **Inputs:**
  - `targetPrim` o `robotPath` (path del prim con Articulation Root API)
  - `jointNames` (token[], orden importa)
  - `velocityCommand` (double[]) o `positionCommand` (double[])
- **Outputs:** ninguno relevante (efectos en PhysX directo).
- **Trap:** el orden de `jointNames` debe coincidir con el orden del
  `velocityCommand` del DiffCtrl ([left, right]).

---

### C. Lidar (RTX sensor pipeline)

#### `Isaac Run One Simulation Frame`

- **Que es:** "Compuerta de un solo disparo". Solo dispara su exec una vez
  por sesion de Play, en el primer frame.
- **Que hace:** Sirve para gatear nodos que solo deben crearse UNA vez
  (como `Isaac Create Render Product`). Si lo pones detras de un nodo que
  crearia un render product nuevo cada frame, ahorras GPU y memoria.
- **Inputs:** `execIn`.
- **Outputs:** `step` (exec, dispara una sola vez).

#### `Isaac Create Render Product`

- **Que es:** Conector entre un sensor (OmniLidar o Camera) y la pipeline
  de render RTX-Sensor.
- **Que hace:** Crea un "hydra texture" en GPU asociado al prim del
  sensor, donde la pipeline RTX escribira la data del sensor (intensidad,
  distancia, etc).
- **Inputs:** `cameraPrim` (target del sensor), `execIn`, `width`, `height`.
- **Outputs:** `execOut`, `renderProductPath` (token, path al render
  product creado).
- **NOTA:** En cargo_bot_ws decidimos NO usar este nodo por bugs de Isaac
  5.1.0-rc.19 — el `publish_lidar.py` lo arma desde Python a mano.

#### `ROS2 RTX Lidar Helper`

- **Que es:** Adapter que convierte la data del RTX Sensor a mensajes ROS 2
  (`sensor_msgs/LaserScan` o `sensor_msgs/PointCloud2`).
- **Que hace:** Adjunta un "writer" al render product del lidar. El writer
  lee los AOVs (`GenericModelOutput`, `RtxSensorMetadata`) y los serializa
  como mensaje ROS 2. Publica en el topic configurado.
- **Inputs:** `context`, `execIn`, `renderProductPath`, `topicName`,
  `frameId`, `type` (`laser_scan` o `point_cloud`), `showDebugView`.
- **Outputs:** ninguno (efecto = mensaje publicado).
- **NOTA:** Igual que arriba, en este proyecto lo armamos desde Python.

---

### D. Odometria + TF + Clock

#### `Isaac Compute Odometry Node`

- **Que es:** Calculador de odometria desde la pose mundial del chassis.
- **Que hace:** Lee la pose (position + orientation) del `chassisPrim` y
  su twist (linear/angular velocity), y los emite. Tipicamente se conecta
  al Publish Odometry para mandarlos por ROS.
- **Inputs:** `chassisPrim` (target, debe ser el Articulation Root o un
  link rigido).
- **Outputs:** `position` (vec3d), `orientation` (quatd), `linearVelocity`
  (vec3d), `angularVelocity` (vec3d).

#### `ROS2 Publish Odometry`

- **Que es:** Publisher ROS 2 de `nav_msgs/Odometry`.
- **Que hace:** Empaqueta los inputs (position/orientation/velocidades) en
  un mensaje Odometry y lo publica en el topic configurado.
- **Inputs:** `context`, `execIn`, `position`, `orientation`,
  `linearVelocity`, `angularVelocity`, `topicName`, `chassisFrameId`,
  `odomFrameId`, `timeStamp`.
- **Outputs:** ninguno.

#### `ROS2 Publish Transform Tree`

- **Que es:** Publisher de TF (todos los frames del robot).
- **Que hace:** Recorre la jerarquia del USD bajo `targetPrims` y publica
  cada xform como un `geometry_msgs/TransformStamped` en `/tf`.
- **Inputs:** `context`, `execIn`, `parentPrim`, `targetPrims`,
  `topicName` (default `tf`).
- **Outputs:** ninguno.

#### `ROS2 Publish Clock`

- **Que es:** Publisher de `rosgraph_msgs/Clock`.
- **Que hace:** Manda el sim time actual al topic `/clock`. ESENCIAL para
  que Nav2, SLAM y otros nodos ROS sepan que estamos en simulacion (con
  `use_sim_time:=true`).
- **Inputs:** `context`, `execIn`, `timeStamp`, `topicName`.
- **Outputs:** ninguno.

#### `Isaac Read Simulation Time`

- **Que es:** Lee el sim time del engine.
- **Que hace:** Cada vez que se evalua, emite el sim time actual (segundos
  desde Play).
- **Inputs:** ninguno.
- **Outputs:** `simulationTime` (double).
- **Cuando usarlo:** Cuando un nodo Publish necesita timestamp del sim
  (no del wall clock). Conectalo a `timeStamp` de Publish Clock,
  Publish Odometry, etc.

---

### E. Patron tipico de armado

Un grafo ROS 2 completo de Isaac Sim para un robot mobile suele tener:

```
                 OnPlaybackTick
                   |
                   tick
       +-----------+-----------+-----------+-----------+-----------+
       |           |           |           |           |           |
       v           v           v           v           v           v
   Subscribe    Publish     Publish    Compute      RunOnce     ReadSimTime
     Twist      Clock       TF Tree    Odometry     |           |
       |           ^           ^          |          v           |
       |        ReadSimTime    |          v       CreateRP       v
       |        ^         (targetPrims=  Publish     |        (todos los
       v        |          /cargo_bot)   Odometry    v         publishers
   BreakVec     |                          ^      RTXLidar      necesitan
       |        +--------------------------+      Helper        timeStamp)
       |              (Context global)        (publica /scan)
       v
   DiffCtrl
       |
       v
   ArticulationController
       |
       v
   [PhysX wheels giran]
```

> **Tip:** No tengas miedo de tener MUCHOS nodos. Cuanto mas modular, mas
> facil de debuggear. La performance no se ve afectada hasta que tenes
> cientos de nodos.
