# cargo_bot_simulation

Paquete Isaac Sim del proyecto `cargo_bot_ws`. Contiene la escena USD del
mundo simulado, scripts Python para correr DENTRO de Isaac, y launchers
Windows `.cmd` para abrir Isaac con el entorno DDS correcto.

> **¿Empezando de cero?** Leé primero
> [`BUILD_SCENE_FROM_SCRATCH.md`](./BUILD_SCENE_FROM_SCRATCH.md) —
> procedimiento autoritativo paso a paso para construir la escena
> `scene_v2.usda` desde el URDF en Isaac Sim 5.1, evitando todas las
> trampas que se acumularon en la `scene.usda` anterior. Usa los helpers
> `scripts/add_lidar.py` y `scripts/publish_lidar.py`.

> Si recién llegás al proyecto, leé también `../../MASTER_PLAN.md`.

---

## Estructura

```
src/cargo_bot_simulation/
├── README.md                ← este archivo
├── scenes/                  ← USD scenes
│   ├── (vacío hasta que se cree scene_v2.usda siguiendo BUILD_SCENE_FROM_SCRATCH.md)
│   └── legacy/              ← escenas corruptas archivadas (NO usar)
│       ├── scene.usda       ← escena original con todos los bugs acumulados
│       └── scene.usda.bak   ← backup pre-edición manual del 2026-05-23
├── BUILD_SCENE_FROM_SCRATCH.md  ⭐ guía autoritativa de construcción
├── scripts/                 ← Python scripts que corren dentro de Isaac
│   ├── add_lidar.py                     ⭐ helper Step 5 de la guía nueva
│   ├── publish_lidar.py                 ⭐ helper Step 8 de la guía nueva
│   ├── standalone_lidar_publisher.py    ← SOLUCIÓN standalone (scene.usda legacy)
│   ├── diag_lidar.py                    ← diagnóstico runtime del pipeline lidar
│   ├── diag_output.txt                  ← último output del diag (gitignore)
│   ├── lidar_python_workaround.py       (intento in-GUI, fallido)
│   ├── lidar_clean_aovs.py              (cleanup agresivo, fallido)
│   ├── fix_lidar.py / fix_lidar_v2.py   (cleanup Replicators, fallidos)
│   └── rewire_helper.py                 (rewire helper.renderProductPath, fallido)
└── launch/                  ← entry points Windows .cmd
    ├── run_standalone_lidar.cmd         ⭐ launcher de la solución
    └── run_rtx_lidar_standalone.cmd     (sanity test: ejemplo bundled NVIDIA)
```

---

## Cómo correr el lidar (estado 2026-05-23)

El RTX Lidar publica `/scan_py` vía un standalone Isaac que carga la escena
en una sesión limpia (sin GUI Kit). El workflow GUI estándar tiene 2 bugs
en `omni.replicator.core` que hacen imposible obtener un render product
limpio (ver memo `isaac51-rtx-lidar-actiongraph`).

**Pasos:**

1. **Cerrá Isaac Sim GUI completamente** (la solución arranca su propia
   sesión Isaac headless; necesita el GPU exclusivo).
2. Asegurate que el Discovery Server esté corriendo en WSL:
   ```
   wsl -d Ubuntu-22.04 -- bash /mnt/c/Users/agusp/cargo_bot_ws/config/start_discovery_server.sh
   ```
3. Doble click en:
   ```
   C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\launch\run_standalone_lidar.cmd
   ```
4. Esperá ~30-90s en la consola hasta ver `[standalone_lidar] PLAYING.`
5. **Dejá esa consola ABIERTA** mientras querés que el lidar publique.

**Verificación** (terminal WSL aparte, con `config/source_ros_wsl.sh`):

```bash
ros2 topic hz /scan_py            # esperado ~10 Hz
ros2 topic echo /scan_py --once   # esperado frame_id="lidar_link", ~1066 ranges
```

---

## Cómo correr el diagnóstico (debug futuro)

Si el lidar deja de publicar y querés ver qué pasa adentro del stage:

1. Con Isaac corriendo (GUI o standalone), abrí Script Editor (Window → Script Editor).
2. **File → Open** → `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\diag_lidar.py`
3. **Run**.
4. Revisá el output en `scripts/diag_output.txt` (mismo directorio).

---

## Historial (por qué hay tantos scripts en `scripts/`)

Durante la depuración de Fase 2 acumulamos varios intentos. Los dejé
como referencia histórica:

| Script | Intento | Resultado |
|--------|---------|-----------|
| `fix_lidar.py` | Borrar 3 Replicator render products huérfanos | RemovePrim no funcionó por `no_delete=true` |
| `fix_lidar_v2.py` | Lo mismo forzando SetActive(False) | Prims no se borraron del USD |
| `rewire_helper.py` | Apuntar helper.renderProductPath al Replicator bueno | 0 msg/s aún apuntando bien |
| `lidar_python_workaround.py` | Bypass del OG con render product Python | `omni.replicator.core` auto-inyecta LdrColor |
| `lidar_clean_aovs.py` | Cleanup total + override de AOVs | Hydra engine quedó con refs colgadas |
| `standalone_lidar_publisher.py` | Carga scene.usda en SimulationApp standalone | ⭐ Funciona |

Detalles técnicos completos en el memo `isaac51_rtx_lidar_actiongraph.md`
en el directorio de memoria de Claude (secciones "Debug log #1, #2, #3").

---

## Convenciones

- **Scripts Python** acá usan el Python embebido de Isaac (`C:\isaacsim_51_ga\python.bat`),
  NO el Python de WSL. Tienen acceso a `omni.usd`, `pxr`, `omni.replicator.core`, etc.
- **Launchers `.cmd`** detectan el IP de WSL y parchean `config/fastdds_isaac.xml`
  antes de lanzar — patrón de `config/launch_isaac_ros.cmd`.
- **Topic name del lidar:** `/scan_py` (NO `/scan` ni `/laser_scan` — para no
  colisionar con el OG path que está roto en Isaac 5.1.0-rc.19).
- **Frame ID:** `lidar_link` (TF tree convention).
- **`ROS_DOMAIN_ID=1`** siempre (cargo_bot_ws convention, NO usar 4 que es del swarm).

---

## Issues abiertos relacionados

Ver `cargo_bot_ws_open_problems.md` (memo de Claude) — P1 (lidar visual mesh)
y P2 (lidar publisher / Hydra cache) están documentados ahí.
