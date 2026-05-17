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
| `/scan` | sensor_msgs/LaserScan | Isaac → WSL2 |
| `/clock` | rosgraph_msgs/Clock | Isaac → WSL2 |
| `/tf` | tf2_msgs/TFMessage | Isaac → WSL2 |
| `/map` | nav_msgs/OccupancyGrid | WSL2 (SLAM) |

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
├── src/
│   ├── cargo_bot_description/       # URDF/Xacro, meshes, RViz configs  [ament_cmake]
│   ├── cargo_bot_bringup/           # Launch files (sim + real)          [ament_python]
│   ├── cargo_bot_hardware/          # ros2_control HW interface (C++)    [ament_cmake]
│   ├── cargo_bot_navigation/        # Nav2 configs, costmaps, planners   [ament_cmake]
│   └── cargo_bot_simulation/        # Isaac Sim USD, OmniGraph configs   [ament_cmake]
└── MASTER_PLAN.md
```

### TF Tree

```
map → odom → base_footprint → base_link
                                  ├── left_wheel_link    (continuous joint)
                                  ├── right_wheel_link   (continuous joint)
                                  ├── caster_wheel_link  (fixed joint)
                                  ├── lidar_link         (fixed joint)
                                  └── imu_link           (fixed joint)
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
| Controller | **MPPI** | Trayectorias suaves, respeta cinematica diferencial. Fallback: RPP si CPU no alcanza en RPi |
| Planner | **Smac2D** | Mejor que NavFn en espacios estrechos (pasillos de casa) |
| SLAM | **slam_toolbox** (async) | Menos CPU que sync, suficiente para indoor |
| Localizacion | **nav2_amcl** | Post-SLAM, mas liviano que seguir corriendo SLAM |
| Recovery | spin, backup, wait | Defaults de Nav2 |
| BT | navigate_to_pose_w_replanning_and_recovery | Default robusto |

### Costmaps (valores iniciales, tunear con robot real)
- **Global costmap:** static layer + inflation layer
- **Local costmap:** obstacle layer (del LiDAR) + inflation layer
- **Inflation radius:** radio_del_robot + 0.1 m (TBD con medidas del CAD)
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

### Fase 3: SLAM
**Objetivo:** Mapa del ambiente simulado.
**Done cuando:** Mapa `.pgm`/`.yaml` guardado, coherente con la escena en RViz2.
**Dependencia:** Fase 2.
**Quien:** Usuario escribe configs y launch. Claude explica parametros.

Entregables:
- `cargo_bot_navigation/config/slam_toolbox.yaml`
- `cargo_bot_bringup/launch/slam.launch.py`
- `cargo_bot_navigation/maps/` (mapas guardados)

### Fase 4: Navegacion Autonoma (Nav2)
**Objetivo:** Robot navega de A a B esquivando obstaculos.
**Done cuando:** Goal en RViz2 → robot llega sin chocar.
**Dependencia:** Fase 3 (mapa).
**Quien:** Usuario escribe y tunea. Claude explica cada parametro.

Entregables:
- `cargo_bot_navigation/config/nav2_params.yaml` (MPPI, Smac2D, costmaps, AMCL)
- `cargo_bot_bringup/launch/navigation.launch.py`

### Fase 5: Logica de Misiones
**Objetivo:** Secuencias de navegacion con logica.
**Done cuando:** "Ir a cocina, esperar 5s, volver a base" funciona end-to-end.
**Dependencia:** Fase 4.
**Quien:** Usuario implementa. Claude guia arquitectura.

Opciones (usuario decide cuando llegue):
- Behavior Tree (nav2_bt)
- State machine (C++ node)

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

## 9. Division de Trabajo (resumen)

| Fase | Claude | Usuario |
|------|--------|---------|
| 0 - DDS Setup | Escribe + ejecuta todo | Verifica |
| 1 - URDF/Xacro | Da recursos y recomendaciones | Hace CAD + escribe URDF |
| 2 - Isaac Sim | Guia si pregunta | Importa URDF, OmniGraph, escena |
| 3 - SLAM | Guia configs, explica params | Escribe YAMLs y launch files |
| 4 - Nav2 | Guia, explica cada param | Escribe y tunea |
| 5 - Misiones | Guia arquitectura | Implementa |
| 6-7 - Hardware | Pendiente | Pendiente |

## 10. Recursos Recomendados

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
