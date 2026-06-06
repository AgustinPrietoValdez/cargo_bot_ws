# Lectures - cargo_bot

Lecciones de aprendizaje del proyecto cargo_bot (Type = Learning). **Formato: HTML** &mdash; abrí
cada `LECTURE_*.html` en el navegador (doble click). No hace falta instalar nada; el quiz y los
diagramas corren solos.

## Como leerlas

Cada lecture sigue el molde: **concepto primero, aplicacion despues**.
- Cajas **azules** = el concepto general (cualquier proyecto ROS2).
- Cajas **verdes** = como lo usamos en cargo_bot.
- Cajas **violetas** = "Deeper" (mas profundidad).
- Cajas **ambar** = traps / gotchas.
- **Diagramas SVG** que muestran quien le habla a quien (topics) y las TF.
- **Quiz autocorregible** al final (elegis, "Check my answers", te corrige + explica + puntaje).

Archivos compartidos: `lectures.css` (estilos) y `lectures.js` (motor del quiz). Cada lecture los
enlaza; para una lecture nueva solo se definen las preguntas del quiz.

## Lecture vs Guia de build

- **Lecture** (`docs/lectures/LECTURE_NN_*.html`) = entender el tema a fondo (teoria + el por que).
- **Guia de build** (`docs/FASE*_GUIA_*.md`) = los pasos exactos para construir y testear.

Regla del proyecto: cada tema = **leccion profunda -> guia de build -> test**.

## Curriculum

Las lectures de fases ya completadas son repaso/consolidacion de lo construido.

| #  | Archivo | Tema | Fase | Estado |
|----|---------|------|------|--------|
| 01 | `LECTURE_01_ros2_from_zero.html` | ROS2 desde cero: el grafo, nodos y sus atributos | 0 | repaso |
| 02 | `LECTURE_02_dds_discovery.html`  | DDS, Discovery Server, DOMAIN_ID, QoS | 0 | repaso |
| 03 | `LECTURE_03_urdf_xacro.html`     | URDF + xacro (links, joints, inercias) | 1 | repaso |
| 04 | `LECTURE_04_isaac_sensors.html`  | Isaac Sim, OmniGraph, ROS2 bridge, sensores | 2 | repaso |
| 05 | `LECTURE_05_slam_ekf.html`       | SLAM, EKF, TF tree | 3 | repaso |
| 06 | `LECTURE_06_nav2_core.html`      | Nav2 core: AMCL, costmaps, NavFn, DWB, lifecycle, BT | 4a | repaso |
| 07 | `LECTURE_07_nav2_tuning.html`    | Nav2 tuning: inflation, critics DWB, recovery, smoothers | 4a.2 | **proximo** |
| 08+| (futuro) | Safety, camara+AprilTag, misiones, HW prep, capstone | 4b+ | futuro |

## Research policy (heredada del project guide)

Solo fuentes oficiales o proyectos conocidos, con los links. Las lectures citan docs.ros.org,
navigation.ros.org / docs.nav2.org, NVIDIA Isaac, fast-dds.docs.eprosima.com, REP-xxx.

> El plan general (Action Plan) y la vision viven en Obsidian
> (`Documentos/Notas/Guides/Cargo_bot_guide.md`), la fuente de verdad del proyecto.
