# Master Plan: Robot de Carga Autonomo en Interiores

Este documento centraliza toda la arquitectura, hardware, software y fases de desarrollo del robot de carga autonomo para tareas domesticas.

---

## 1. Especificaciones Generales

| Spec | Valor |
|------|-------|
| Traccion | Diferencial (2 ruedas motrices + 1 caster) |
| Capacidad de carga | 5 kg |
| Alimentacion | LiPo 3S (11.1V) / 2000 mAh (prototipo, espacio para expansion) |
| Sensor percepcion | LiDAR 2D (RPLidar A1/A2) — SLAM + obstaculos |
| Sensor odometria | Encoders de cuadratura + IMU (MPU6050 o BNO085) |
| Procesamiento bajo nivel | STM32 (C/C++) — PID, encoders, IMU |
| Procesamiento alto nivel | Raspberry Pi 4/5 — ROS 2 Humble |
| Velocidad maxima | TBD (depende de motores elegidos) |

## 2. Entorno de Desarrollo

| Recurso | Detalle |
|---------|---------|
| GPU | RTX 4060 Laptop, 8 GB VRAM |
| RAM | 16 GB, 16 threads |
| Isaac Sim | 5.1.0 (Windows nativo) |
| ROS 2 | Humble (WSL2 Ubuntu 22.04) |
| RMW | rmw_fastrtps_cpp (ambos lados) |
| DDS | Discovery Server — WSL2 como SERVER, Isaac Sim como CLIENT |
| ROS_DOMAIN_ID | 1 |

### Arquitectura de comunicacion

```
┌─────────────────────────┐     DDS (UDPv4)     ┌────────────────────────────┐
│   Windows (Isaac Sim)   │◄───────────────────►│   WSL2 (ROS 2 Humble)      │
│                         │   ROS_DOMAIN_ID=1    │                            │
│  - Fisicas + render     │                      │  - Nav2                    │
│  - RTX LiDAR virtual    │   Fast RTPS          │  - slam_toolbox            │
│  - Differential ctrl    │   CLIENT ──► SERVER  │  - robot_localization      │
│  - ROS 2 Bridge (built-in)                     │  - teleop / misiones       │
└─────────────────────────┘                      └────────────────────────────┘
```

### Topics principales

| Topic | Tipo | Direccion |
|-------|------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | WSL2 → Isaac |
| `/odom` | nav_msgs/Odometry | Isaac → WSL2 |
| `/odometry/filtered` | nav_msgs/Odometry | EKF (post robot_localization) |
| `/imu/data` | sensor_msgs/Imu | Isaac → WSL2 |
| `/scan` | sensor_msgs/LaserScan | Isaac → WSL2 |
| `/clock` | rosgraph_msgs/Clock | Isaac → WSL2 |
| `/tf` | tf2_msgs/TFMessage | Isaac → WSL2 |
| `/map` | nav_msgs/OccupancyGrid | WSL2 (SLAM) |

### Operator Interface (cómo le decimos al robot qué hacer)

Staging progresivo conforme avanzamos:

| Etapa | Interfaz | Cuándo |
|-------|----------|--------|
| Desarrollo | `ros2` CLI desde laptop (publicar a topics, action calls) | Hoy y hasta que SLAM+Nav2 estén estables |
| Pruebas más reales | Foxglove Studio con paneles custom (botones publish, plots, mapa) | Una vez que tengamos camera y misiones funcionando |
| Producto final | App tablet (PWA o nativa) → REST API HTTP → mission_runner | Después de Nav2 + Mission DSL (ver Fase 5) |

Voice commands (whisper/vosk) evaluado y descartado por ahora.

## 3. Diseno Mecanico (CAD)

El chasis se modela para fabricacion aditiva, dividido en 3 pisos:

- **Piso 1 (Base Motriz):** Motorreductores (ej. JGB37-520), ruedas motrices, caster wheel metalica, bateria LiPo (centro de gravedad bajo). El eje motriz debe quedar alineado debajo del centro de la bandeja de carga.
- **Piso 2 (Electronica):** SBC + PCB custom (KiCad). Ventilacion + standoffs M3.
- **Piso 3 (Percepcion + Carga):** Superficie reforzada para 5 kg. LiDAR 2D elevado sin obstrucciones.

Dimensiones: TBD (salen del CAD).

## 4. Diseno de PCB y Potencia

- **Esquematico KiCad:** STM32 + conectores encoders + I2C/SPI para IMU.
- **Drivers de motor:** VNH5019 o TB6612FNG (para traccionar 7.5 kg totales).
- **Gestion de energia:** Step-Down (Buck) 5A: 11.1V → 5V para LiDAR y RPi.

## 5. Workspace ROS 2

```
cargo_bot_ws/
├── config/                          # DDS configs, FastDDS XMLs
├── docs/                            # Guías por fase (FASE1, FASE2, FASE3, ...)
├── src/
│   ├── cargo_bot_description/       # URDF/Xacro, meshes, RViz configs    [ament_cmake]  ← existe
│   ├── cargo_bot_simulation/        # Isaac Sim USD, OmniGraph, scripts   [ament_cmake]  ← existe
│   ├── cargo_bot_navigation/        # SLAM, EKF, Nav2, costmaps, maps     [ament_cmake]  ← Fase 3
│   ├── cargo_bot_bringup/           # Launch files (localization, slam, nav2) [ament_python] ← Fase 3
│   ├── cargo_bot_safety/            # Watchdog, tip-over, collision shield [ament_cmake]  ← Fase 4b
│   ├── cargo_bot_behaviors/         # Visual servoing, cargo state machine, hatch, LEDs [ament_cmake] ← Fase 4c+5
│   ├── cargo_bot_missions/          # Mission DSL parser, runner, scheduler [ament_python] ← Fase 5
│   ├── cargo_bot_api/               # REST API frontend                    [ament_python] ← Fase 5
│   └── cargo_bot_hardware/          # ros2_control HW interface (STM32)    [ament_cmake]  ← Fase 6
└── MASTER_PLAN.md
```

### TF Tree

```
map → odom → base_footprint → base_link
                                  ├── left_wheel_link        (continuous joint)
                                  ├── right_wheel_link       (continuous joint)
                                  ├── caster_wheel_link      (fixed joint)
                                  ├── lidar_link             (fixed joint)
                                  ├── imu_link               (fixed joint)
                                  ├── camera_link            (fixed joint, Fase 4c)
                                  │   └── camera_link_optical (fixed joint, REP-105)
                                  ├── hatch_link             (revolute joint, Fase 5)
                                  └── status_led_link        (fixed joint, Fase 5)
```

### Flujo de datos (simulacion)

```
Nav2 (WSL2)                     Isaac Sim (Windows)
    │                                │
    ├─ calcula ruta ──────────►      │
    ├─ publica /cmd_vel ─────►  ROS2 Subscribe Twist
    │                           Differential Controller
    │                           Articulation Controller
    │                                │
    │  ◄──── /odom ────────────  ROS2 Publish Odometry
    │  ◄──── /scan ────────────  ROS2 Publish LaserScan (RTX Lidar)
    │  ◄──── /tf ──────────────  ROS2 Publish TF
    │  ◄──── /clock ───────────  ROS2 Publish Clock
    │                                │
robot_localization (WSL2)            │
    ├─ fusiona odom + IMU            │
    └─ publica odom → base_link      │
```

### Flujo de datos (hardware real)

```
Nav2 (RPi)                      STM32
    │                                │
    ├─ publica /cmd_vel ─────►  ros2_control HW Interface
    │                           Serial UART (protocolo TBD)
    │                           ──────────────────────────►  PID + PWM
    │                                                        Lee encoders
    │                           ◄──────────────────────────  Ticks + IMU
    │  ◄──── /odom             ros2_control publica odom
    │                                │
robot_localization (RPi)             │
    ├─ fusiona odom + IMU            │
    └─ publica odom → base_link      │
```

## 6. Configuracion Isaac Sim

### Performance (8 GB VRAM)
- Render: 720p o menos
- RTX Lidar: ~360 rays (emulando RPLidar A1)
- Desactivar ray-traced reflections/GI si lagea
- Un solo robot en escena

### LiDAR simulado (emulando RPLidar A1)
| Param | Valor |
|-------|-------|
| Samples por scan | 360 |
| Frecuencia | 5.5 Hz |
| Rango min | 0.15 m |
| Rango max | 12.0 m |
| FOV | 360 deg |

### OmniGraph (Action Graph)
Nodos necesarios:
1. `On Playback Tick` (trigger)
2. `ROS2 Context` (bridge)
3. `ROS2 Subscribe Twist` → `/cmd_vel`
4. `Differential Controller` → velocidades de rueda
5. `Articulation Controller` → aplica a joints
6. `ROS2 Publish Odometry` → `/odom`
7. `ROS2 Publish LaserScan` → `/scan`
8. `ROS2 Publish Transform Tree` → `/tf`
9. `ROS2 Publish Clock` → `/clock`

### Escena
- Fase inicial: habitacion simple (4 paredes + cubos como obstaculos)
- Fase posterior (opcional): recrear espacio real del usuario

## 7. Navegacion (Nav2)

| Componente | Eleccion | Razon |
|------------|----------|-------|
| Controller | **DWB** (default Humble) | El combo estandar + mas documentado; hace evasion local real (base para obstaculos dinamicos). MPPI = upgrade futuro si queremos trayectorias mas optimas; RPP = plan B si DWB se vuelve molesto de tunear |
| Planner | **NavFn** (default Humble) | El mas estandar/probado, cero tuneo, nunca falla en encontrar path. El pivoteo en esquinas se ataca con smoother_server (Step 2), NO cambiando de planner. Smac2D = swap futuro si queremos paths mas suaves nativos |
| SLAM | **slam_toolbox** (async) | Menos CPU que sync, suficiente para indoor |
| Localizacion | **nav2_amcl** | Post-SLAM, mas liviano que seguir corriendo SLAM. Pose inicial via `set_initial_pose` (sim spawnea en pose conocida) |
| Recovery | spin, backup, drive_on_heading, assisted_teleop, wait | Defaults de Nav2 Humble (tipos plugin forma `nav2_behaviors/...`) |
| BT | navigate_to_pose_w_replanning_and_recovery | Default robusto |

> Decision tomada 2026-05-30 tras research-fix validado contra el branch Humble de Nav2. Guia operativa completa: [`docs/FASE4_GUIA_NAV2.md`](docs/FASE4_GUIA_NAV2.md). El plan original decia MPPI+Smac2D; se cambio a DWB+NavFn por ser el combo estandar y mas seguro para el primer bringup en un cuarto chico (5x4.9m). MPPI/Smac2D quedan como upgrades futuros (swap de un bloque de config).

### Costmaps (valores iniciales, tunear con robot real)
- **Global costmap:** static layer + inflation layer
- **Local costmap:** obstacle layer (del LiDAR) + inflation layer
- **Inflation radius:** **0.35 m** (≈ robot_radius 0.18 + margen). NO usar el default 0.55 m: en un cuarto de 5 m la inflacion desde paredes opuestas se solapa en el centro y el planner no encuentra corredor libre
- **Resolucion:** 0.05 m

## 8. Fases de Desarrollo

### Fase 0: DDS Discovery Server Setup
**Objetivo:** Isaac Sim y ROS 2 se comunican via DDS.
**Done cuando:** `ros2 topic list` en WSL2 muestra topics de Isaac Sim con escena en Play.
**Quien:** Claude hace todo. Usuario verifica.

Archivos:
- `config/fastdds_isaac.xml` — CLIENT profile para Windows
- `config/fastdds_wsl.xml` — SERVER + UDPv4 profile para WSL2
- `config/start_discovery_server.sh` — arranca Discovery Server en WSL
- `config/launch_isaac_ros.cmd` — detecta IP WSL, setea env vars, lanza Isaac Sim
- Regla firewall: UDP inbound puertos 7400-7420 + 11811

### Fase 1: Robot Description (URDF/Xacro)
**Objetivo:** Modelo URDF completo con TF tree correcto.
**Done cuando:** Robot visible en RViz2, ruedas giran con `joint_state_publisher_gui`, frames correctos.
**Dependencia:** CAD del usuario con medidas finales.
**Quien:** Usuario hace todo. Claude da recursos para aprender.

Entregables:
- `cargo_bot_description/urdf/cargo_bot.urdf.xacro` (modelo principal)
- `cargo_bot_description/urdf/macros.xacro` (macros de inercia)
- `cargo_bot_description/launch/display.launch.py` (RViz2 viewer)
- `cargo_bot_description/rviz/cargo_bot.rviz` (config RViz)
- `cargo_bot_description/meshes/` (si usa STL del CAD)

### Fase 2: Isaac Sim Integration
**Objetivo:** Robot se mueve en Isaac controlado desde WSL2.
**Done cuando:** `teleop_twist_keyboard` mueve robot, `/odom` y `/scan` publican datos coherentes.
**Dependencia:** Fase 0 + Fase 1.
**Quien:** Usuario hace todo en la UI de Isaac Sim. Claude guia si hay dudas.

Pasos:
1. URDF Importer → genera USD
2. Configurar ArticulationRoot + Drive en joints de ruedas
3. Agregar RTX Lidar (360 samples, 5.5 Hz, 12 m)
4. Crear Action Graph (OmniGraph) segun seccion 6
5. Escena simple: 4 paredes + cubos
6. Play + teleop desde WSL2

### Fase 3: SLAM (incluye IMU + EKF como prerequisito)
**Objetivo:** Mapa del ambiente simulado de alta calidad usando lidar 2D + odometría fusionada con IMU.
**Done cuando:** Mapa `.pgm`/`.yaml` guardado, coherente con la escena en RViz2 sin desalineaciones por drift de heading.
**Dependencia:** Fase 2 (scene_v3 con lidar + odom + tf funcionando).
**Quien:** Usuario escribe configs y launch. Claude explica parametros.
**Guía detallada:** `docs/FASE3_GUIA_SLAM.md` (paso-a-paso completo).

Pasos resumidos:
1. **IMU + EKF setup** (prerequisito antes de SLAM)
   - Agregar `<sensor type="imu">` al xacro + inertial al `imu_link`
   - Crear `scene_v4.usda` con IMU Sensor OmniGraph publicando `/imu/data` ~100 Hz
   - Borrar TODO el `ROS_TF` graph de Isaac y delegar al `robot_state_publisher` (WSL) como autoridad del subtree URDF `base_footprint → base_link → {wheels, lidar, imu}`
   - EKF queda como única authority de `odom → base_footprint`
   - Crear packages `cargo_bot_navigation` (config) y `cargo_bot_bringup` (launches)
   - `ekf.yaml` fusiona `/odom` (vx, vyaw) + `/imu/data` (yaw rate, ax) → `/odometry/filtered`
   - `localization.launch.py` levanta los tres nodos juntos: `robot_state_publisher` + `joint_state_publisher` + `ekf_filter_node`
2. **slam_toolbox** (mapeo)
   - Config tuneada para nuestro lidar (range_max=12m, frame_id=lidar_link, base_frame=base_footprint)
   - `slam.launch.py` con `use_sim_time:=true`
   - Mapear manual (teleop) o automático (frontier explorer, ver roadmap futuro)
3. **Guardar mapa** (`.pgm` + `.yaml` en `cargo_bot_navigation/maps/`)

Entregables:
- `cargo_bot_description/urdf/sensors.xacro` (upgrade imu_link)
- `cargo_bot_simulation/scenes/scene_v4.usda` (con IMU OmniGraph)
- `cargo_bot_navigation/config/ekf.yaml`
- `cargo_bot_navigation/config/slam_toolbox.yaml`
- `cargo_bot_bringup/launch/localization.launch.py`
- `cargo_bot_bringup/launch/slam.launch.py`
- `cargo_bot_navigation/maps/` (mapas guardados)

### Fase 4a: Navegacion Autonoma (Nav2 core, sin safety layers)
**Objetivo:** Robot navega de A a B esquivando obstaculos del mapa, en condiciones limpias.
**Done cuando:** Goal en RViz2 → robot llega sin chocar (escena sin obstáculos dinámicos, sin perturbaciones).
**Dependencia:** Fase 3 (mapa).
**Quien:** Usuario escribe y tunea. Claude explica cada parametro.

Por qué split en 4a/4b: meter Nav2 + safety layers al mismo tiempo hace que cualquier bug ("robot freeza random") sea ambiguo de debuggear — ¿es Nav2 mal tuneado o safety overreactor? Validamos Nav2 puro primero, después capas de seguridad encima.

Pasos:
1. Nav2 core: **DWB controller**, **NavFn planner**, **AMCL localizer** (ver `docs/FASE4_GUIA_NAV2.md`)
2. Costmaps (global + local) con obstacle layer + inflation layer
3. Recovery behaviors default (spin, backup, wait)
4. `twist_mux` configurado desde el inicio con UN input (Nav2) — preparado para sumar inputs en 4b sin refactor
5. RViz2 config para enviar goals + visualizar plan + costmap

Entregables:
- `cargo_bot_navigation/config/nav2_params.yaml` (MPPI, Smac2D, costmaps, AMCL)
- `cargo_bot_navigation/config/twist_mux.yaml` (con un solo input por ahora)
- `cargo_bot_bringup/launch/navigation.launch.py`

### Fase 4b: Safety Layers (encima de Nav2 estable)
**Objetivo:** Robot resistente a fallos de Nav2, obstáculos dinámicos, tip-over, batería baja.
**Done cuando:** Tests de cada safety feature pasan individualmente (ver cada item).
**Dependencia:** Fase 4a (Nav2 funcionando limpio).
**Quien:** Usuario escribe. Claude explica trade-offs y trigger conditions.

Pasos:
1. **Geofencing virtual** — `costmap_2d` plugin con polígonos prohibidos en YAML (escaleras, zonas off-limits). Test: goal del otro lado de un polígono → Nav2 NO planifica cruzando.
2. **Pre-Nav2 collision shield** — nodo independiente que escucha `/scan`, si min_range < threshold en front-arc fuerza zero-twist via twist_mux (segundo input). Test: obstáculo aparece de golpe → robot freeza antes de chocar.
3. **`cargo_bot_safety` package — watchdog** — escucha `/cmd_vel`, si no llega nada en >0.5s publica zero-twist. Plus topic `/emergency_stop` (Bool latched) que cuando True bloquea todo. Test: matar Nav2 mid-traversal → robot se queda quieto.
4. **Tip-over detector** — `|roll|` o `|pitch|` > 30° → `/emergency_stop=true` + log a `/diagnostics`. Test: forzar inclinación en sim → emergency stop dispara.
5. **Battery simulation node** (solo sim) — drena `sensor_msgs/BatteryState` según módulo cmd_vel, recarga si pose ∈ zona-dock. Test: dejar el robot operando, ver curva de drenaje.

Entregables:
- `cargo_bot_navigation/config/geofence.yaml`
- `cargo_bot_navigation/config/twist_mux.yaml` (extendido con collision_shield)
- `cargo_bot_safety/` package nuevo (watchdog, tip-over, collision_shield, emergency_stop manager)
- `cargo_bot_simulation/scripts/battery_sim_node.py`

### Fase 4c: Cámara RGB + AprilTag docking
**Objetivo:** Robot con cámara RGB publicando `/camera/image_raw`, demo de approach-dock con AprilTag para precisión sub-cm.
**Done cuando:** Robot navega al área de un dock virtual con Nav2 (~10cm precisión), después se alinea con AprilTag hasta sub-cm.
**Dependencia:** Fase 4a (Nav2 funcionando) + cámara montada en xacro.
**Quien:** Usuario implementa. Claude explica frames REP-105, AprilTag library, visual servoing math.

Diseño futuro-proof: frames `camera_link` + `camera_link_optical` (REP-105) de entrada — cuando se swappee a RGB-D solo se SUMAN topics depth, no breaking changes.

Pasos:
1. Agregar `<link name="camera_link">` + `<link name="camera_link_optical">` a xacro
2. Regenerar URDF + crear `scene_v5.usda` con Camera prim + ROS2 Camera Helper OmniGraph
3. Verificar feed con `rqt_image_view` o Foxglove
4. Instalar `apriltag_ros`, pegar markers en escena Isaac
5. Behavior tree: `NavigateToPose` (Nav2 al dock) → `visual_servo` (alineación fina por marker)

Entregables:
- `cargo_bot_description/urdf/sensors.xacro` (extendido con camera_link + camera_link_optical)
- `cargo_bot_simulation/scenes/scene_v5.usda`
- `cargo_bot_navigation/config/apriltag_config.yaml`
- `cargo_bot_navigation/behavior_trees/approach_dock.xml`
- `cargo_bot_behaviors/` package nuevo con `visual_servo_node`

### Fase 5: Lógica de Misiones (Mission DSL + Cargo handling)
**Objetivo:** Misiones complejas declarativas en YAML, ejecución autónoma, operator interface estable.
**Done cuando:** Misión `laundry_pickup_simulation` (versión sin ascensor) corre end-to-end desde tablet via REST API.
**Dependencia:** Fase 4c (Nav2 + camera + AprilTag funcionando).
**Quien:** Usuario implementa. Claude guia arquitectura del DSL + REST.

Regla del proyecto recordada acá: **primero que el robot ande (Fases 3-4 completas), después alto nivel (esta fase).**

Pasos:
1. **Mission DSL YAML** — parser que lee archivos tipo `go_to → open_hatch → wait_for_load → close_hatch → go_to → drop`
2. **Cargo state machine** — node que mantiene state {IDLE, APPROACHING, OPENING_HATCH, WAITING_FOR_LOAD, CLOSING_HATCH, TRANSPORTING, DROPPING, DONE} y publica `/cargo_status`
3. **Hatch (tapa superior abrible)** — joint revolute en xacro + action server `OpenHatch`/`CloseHatch`
4. **Mission scheduler/cron** — lee `schedules.yaml` y dispara misiones por tiempo
5. **REST API frontend** — `fastapi` HTTP endpoint para `POST /mission`
6. **App tablet** (futuro, etapa final operator interface) — PWA que apunta al REST API
7. **LED status indicators** — material emisivo controlable + nodo Python que escribe color según estado

Entregables:
- `cargo_bot_missions/` package (parser DSL + runner + scheduler)
- `cargo_bot_missions/missions/*.yaml` (biblioteca de misiones)
- `cargo_bot_missions/schedules.yaml`
- `cargo_bot_api/` package (REST API)
- `cargo_bot_description/urdf/hatch.xacro` (joint revolute + actuator)
- `cargo_bot_behaviors/` package extendido con `hatch_controller`, `cargo_state_machine`, `led_status_publisher`

### Fase 6: Hardware (STM32 + Motores + PCB)
**Objetivo:** Electronica funcionando en banco.
**Done cuando:** Motores giran con PID correcto, encoders reportan ticks.
**Dependencia:** Diseno electronico del usuario.
**Quien:** Pendiente.

Decisiones pendientes:
- Modelo STM32
- Motores (RPM, torque, encoder CPR)
- micro-ROS vs protocolo serial custom
- Protocolo serial (formato frame, comandos)

### Fase 7: Integracion Real
**Objetivo:** Robot fisico navega con Nav2.
**Done cuando:** Robot real navega de A a B en el espacio fisico.
**Dependencia:** Fase 1 + Fase 6.
**Quien:** Pendiente.

Entregables:
- `cargo_bot_hardware/` — ros2_control plugin (C++)
- `cargo_bot_bringup/launch/real_robot.launch.py`
- Parametros Nav2 retuneados para hardware real

## 9. División de Trabajo

### Regla general (aplica a todo el proyecto desde Fase 1+)

**Vos hacés. Claude guía.**

- **Claude:** explica cada paso con desglose pieza-por-pieza (qué hace cada atributo, por qué ese valor, qué pasaría sin él), da snippets exactos para copiar, da comandos exactos para correr, troubleshootea cuando algo rompe, explica cada parámetro de config.
- **Usuario:** ejecuta `ros2 pkg create`, edita los archivos con su editor, opera Isaac GUI, corre los tests de verificación, copia/pega snippets, instala dependencies.

Esta regla aplica a snippets de código (xacro/YAML/Python/C++), comandos de terminal (ros2/colcon/xacro/apt/git), Y operaciones en Isaac Sim (Tools menu, OmniGraph nodes, prims, Property panel). Sin desglose explicativo, sólo es paste-fodder y el usuario no aprende.

Excepción: meta-trabajo de organización de docs/archivos (reorganizar markdown, mover archivos entre carpetas, actualizar índices) — ahí Claude puede tomar la herramienta directamente.

### Tabla resumen por fase

| Fase | Claude | Usuario |
|------|--------|---------|
| 0 - DDS Setup | Escribe + ejecuta todo | Verifica |
| 1 - URDF/Xacro | Da recursos y recomendaciones | Hace CAD + escribe URDF |
| 2 - Isaac Sim | Guía si pregunta | Importa URDF, OmniGraph, escena |
| 3 - SLAM (+IMU/EKF) | Guía configs, explica params | Escribe YAMLs y launch files, ejecuta Isaac |
| 4a - Nav2 core | Guía, explica cada param | Escribe y tunea |
| 4b - Safety layers | Guía trigger conditions + trade-offs | Implementa nodos safety |
| 4c - Camera + AprilTag | Guía frames REP-105 + visual servoing | Implementa |
| 5 - Misiones | Guía arquitectura DSL + REST | Implementa |
| 6-7 - Hardware | Pendiente | Pendiente |

## 10. Tarea Norte (North-Star Task)

**Llevar ropa al lavadero del edificio subiéndose al ascensor.**

Esta es la tarea concreta que guía priorización del proyecto. Cuando una decisión de arquitectura es ambigua, se resuelve preguntando "¿esto acerca o aleja al robot de hacer la laundry-pickup vía ascensor?".

### Requirements upstream que impone

| Requirement | Por qué | Fase donde aterriza |
|-------------|---------|---------------------|
| Multi-floor / multi-map support | Edificio tiene varios pisos, el robot necesita N mapas + waypoints de "elevator" | Post-Fase 5, ver Roadmap Futuro |
| Manipulador físico para botones de ascensor | El ascensor del edificio NO es IoT → MQTT bridge descartado → no hay forma de llamarlo software-only. Robot necesita SÍ o SÍ apretar el botón físico | Post-Fase 5 (es un epic propio) |
| Localización cross-floor | El robot necesita saber en qué piso está sin GPS. Opciones: barómetro, WiFi fingerprint, NFC/RFID por piso, o input manual del operador | Post-Fase 5 |
| TF behavior durante transporte pasivo | Robot quieto dentro del ascensor mientras "el mundo se mueve" alrededor de él (cambia el piso pero TF base_link no se mueve respecto a base_footprint) | Post-Fase 5 |
| Geofencing duro para escaleras | Si el robot está en piso N y planifica goal en piso N+1, NO puede intentar bajar/subir por escalera. Cero tolerancia | Fase 4b (geofencing virtual) |
| Person detection robusta | El ascensor puede estar ocupado por personas | Fase 4c (con camera + YOLO) |
| Cargo handling (hatch + state machine) | La ropa va DENTRO del robot, hatch abrible para cargarla/descargarla | Fase 5 |

### Sub-tarea simplificada (validable antes de la real)

`laundry_pickup_simulation`: mismo flujo pero todo en un solo piso (sin ascensor). Sirve para validar cargo handling + Nav2 + AprilTag dock antes de meter el ascensor encima. Es el "done cuando" de Fase 5.

La versión completa con ascensor es post-Fase 5 (queda en Roadmap Futuro).

## 11. Roadmap Futuro

Features que NO entran en las fases 0-5 estructuradas pero quedan documentadas como follow-ups potenciales. Cuando se cierre Fase 5 y el robot ande end-to-end, decidimos qué priorizar de acá.

### A) Capacidades del robot extendidas

- **Gripper / manipulador 2-3 DOF** ⭐ **habilitante de la north-star task laundry+ascensor**. Alternativas: brazo full con MoveIt2, o button-pusher dedicado (solenoide + servo, o stick rígido + alineación visual via AprilTag). Decisión cuando lleguemos.
- **Status LEDs en el chassis real** (WS2812B strip) — el sim ya lo tendrá en Fase 5, en HW real toca diseñar el circuito.

### B) Navegación y mapeo extendidos

- **Multi-floor / multi-map support** ⭐ **habilitante de la north-star task** — config con N mapas + waypoints elevator + behavior `TakeElevator`.
- **Frontier explorer** (`explore_lite`) — auto-mapping sin teleop, mientras hay SLAM activo encuentra fronteras y manda goals a Nav2 hasta cubrir todo.
- **Speed-by-zone costmap** — `max_vel_x` distinto por zona (slow en pasillos angostos, fast en open space). Evaluar después de probar en ambientes reales.
- **Custom Nav2 BT plugins** — `ApproachDock`, `OpenHatch`, `WaitForOperator`, `ChargeBattery` como action nodes para reutilizar en behavior trees.

### C) Observabilidad y dev experience

- **Foxglove Studio bridge** — UI más fluida que RViz, paneles custom, replay de bags. Setup ~5 min, alta utilidad inmediata.
- **`/diagnostics` aggregator** — `diagnostic_msgs/DiagnosticArray` con health de sensores, compatible con `rqt_robot_monitor`.
- **Structured logger (JSON)** — JSONL a `/tmp/cargo_bot_logs/`, parseable por scripts post-mortem y por LLMs.
- **`rosbag2` record launch** — graba topics seleccionados para debugging + datasets.
- **Prometheus + Grafana** (al final) — exporter ROS → time-series con trends.
- **Web dashboard custom (Flask)** — fallback si Foxglove queda corto para algún caso específico.

### D) Testing / CI / docs

- **`launch_testing` integration tests** — tests Python que arrancan nodos, mandan inputs, verifican outputs.
- **GitHub Actions CI** — workflow que en cada push hace `colcon build` + corre tests.
- **VS Code workspace** — `.vscode/` con tasks, extensiones recomendadas, settings de linting.
- **README básico** ⭐ **prioridad cercana** — cómo levantar el proyecto, comandos básicos, paths importantes.
- **README tutorial-style** (post-Nav2) — diagramas, GIFs, apuntado a portfolio.
- **Architecture diagrams Mermaid** ⭐ **prioridad cercana** — TF tree, node graph, launch hierarchy embebidos en docs.
- **Demo videos generados** — script que toma rosbag y genera mp4 con overlay (mapa + pose + scan + cmd_vel chart).

### E) Simulación realista

- **Multi-scene library** + **escena USD de la casa real con ascensor** — para validar la north-star task en sim antes de HW.
- **Dynamic obstacles** — agentes que se mueven en lazo (persona/mascota cruzando) + objetos estáticos nuevos entre runs (silla movida, juguete en el piso) para forzar local-costmap-only obstacle handling.
- **Domain randomization** (al final) — si decidimos entrenar modelo de vision custom.
- **Physically-accurate friction** (maybe) — si aparecen problemas de patinaje en HW real, evaluar.

### F) Hardware prep (camino a Fases 6-7)

- **`ros2_control` migration** — Fase 5/6, cerca de HW real. `STM32SerialHardware` C++ heredando de `hardware_interface::SystemInterface`. El stack Nav2/SLAM/EKF se queda intacto.
- **micro_ROS skeleton** — proyecto CubeMX con freeRTOS + `micro_ros_stm32`. Permite cerrar comms USB↔ROS antes de tener motores reales.
- **PCB schematic review por Claude** — segunda opinión al esquemático que diseñe el usuario.

### Ideas evaluadas y descartadas

A.5 voice commands (whisper/vosk), B.2 floor stain detection, B.3 QR code, E.5 Slack/Discord alerts, F.4 MQTT bridge (sin ascensor IoT en el edificio del usuario), G.5 sun/lighting variation, H.4 power management (supercap), I.3 dev container/Docker, J.4 rosdoc2 auto-docs.

## 12. Recursos Recomendados

### ROS 2 Fundamentals
- Docs oficiales ROS 2 Humble: https://docs.ros.org/en/humble/Tutorials.html
- URDF/Xacro tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

### Isaac Sim
- Docs oficiales Isaac Sim 5.1: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- URDF Importer: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/import_urdf.html
- OmniGraph ROS2: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/ros2_omnigraph.html

### Nav2
- Nav2 docs: https://docs.nav2.org/
- MPPI controller: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- Smac2D planner: https://docs.nav2.org/configuration/packages/configuring-smac-2d.html

### slam_toolbox
- Repo + docs: https://github.com/SteveMacenski/slam_toolbox
