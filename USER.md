# cargo_bot - USER

Lo que necesito saber para operar el proyecto en el dia a dia.

## Dia a dia: que falta arreglar

Cada problema tiene su issue en GitHub (`AgustinPrietoValdez/cargo_bot_ws`). Esta tabla es el
espejo; la fuente de verdad es el issue.

| Issue | Problema | Estado | Que me toca a mi |
|-------|----------|--------|------------------|
| [#8](https://github.com/AgustinPrietoValdez/cargo_bot_ws/issues/8) | `lidar.stl` (Fusion 360) no se importa en Isaac; el puck queda invisible (no bloquea) | ABIERTO | Re-exportar el STL con header estandar (o convertir a OBJ/USD) cuando quiera |
| [#9](https://github.com/AgustinPrietoValdez/cargo_bot_ws/issues/9) | `docs/FASE2_GUIA_ISAAC_SIM.md` sec4-5 desactualizadas (Isaac 4.x) | ABIERTO | Nada (docs = Claude); pedirselo cuando moleste |
| [#10](https://github.com/AgustinPrietoValdez/cargo_bot_ws/issues/10) | FASE3 sec6: falta nota de revertir yaw anchor con MPU6050 real | ABIERTO | Nada (docs = Claude); relevante recien en Fase 6 HW |

## Como marcar un issue como resuelto (asi Claude se entera)

1. Cerrar el issue **con un comentario corto de como quedo**:
   ```bash
   gh issue close <N> -R AgustinPrietoValdez/cargo_bot_ws -c "Resuelto: <que hice>"
   ```
2. **No tocar la tabla de arriba**: Claude la sincroniza al levantar el proyecto
   (`gh issue list --state all`) y actualiza tambien su `AI.md`.
3. Problema nuevo: `gh issue create` o pedirselo a Claude.

## Comandos del dia

```bash
# Ver mis tareas del proyecto (desde Windows)
python "C:\Users\agusp\Documentos\Organization_App\calendar-app\tools\plan_cli.py" show -p CARGO_BOT
python "C:\Users\agusp\Documentos\Organization_App\calendar-app\tools\plan_cli.py" task "tuning"
```

## Boot completo del stack (navegacion sobre cuarto_v1)

**1. Windows:** doble click `config\launch_all.cmd` (o `launch_isaac_ros.cmd`) -> en Isaac abrir
`src\cargo_bot_simulation\scenes\scene_v4.usda` -> Play.

**2. WSL terminal A** (base: RSP + JSP + EKF):
```bash
cd /mnt/c/Users/agusp/cargo_bot_ws && source config/source_ros_wsl.sh && source install/setup.bash
ros2 launch cargo_bot_bringup localization.launch.py
```

**3. WSL terminal B** (scan fixer; va aparte porque no esta en localization/navigation.launch.py):
```bash
cd /mnt/c/Users/agusp/cargo_bot_ws && source config/source_ros_wsl.sh && source install/setup.bash
ros2 run cargo_bot_bringup scan_angle_fixer --ros-args -p use_sim_time:=true
```

**4. WSL terminal C** (Nav2: map_server + AMCL + stack):
```bash
cd /mnt/c/Users/agusp/cargo_bot_ws && source config/source_ros_wsl.sh && source install/setup.bash
ros2 launch cargo_bot_bringup navigation.launch.py
```

**5. RViz** (aparte, para ver particulas y mandar goals):
```bash
cd /mnt/c/Users/agusp/cargo_bot_ws && source config/source_ros_wsl.sh && source install/setup.bash
rviz2
```
- En el display de `/particle_cloud`: Reliability = **Best Effort** (si no, no se ve la nube).
- Goal con "2D Goal Pose" sobre el mapa.

**Verificacion rapida:** `ros2 topic hz /scan_fixed` (~10 Hz) y `ros2 topic echo /amcl_pose --once`.

**Gotchas:**
- **Stop/Play en Isaac resetea `/clock` y mata el EKF** -> Ctrl+C en las terminales y relanzar.
- Spawn es (0,0,0); AMCL arranca con initial_pose en 0,0,0, no hace falta setearla.
- Detalle fino y troubleshooting: `docs/FASE4_GUIA_NAV2.md` sec6-8.

## Donde vive cada cosa

| Cosa | Path |
|------|------|
| Guia del proyecto (Obsidian, fuente de verdad) | `Documentos\Notas\Guides\Cargo_bot_guide.md` |
| Notas de Claude (estado + protocolos) | `AI.md` (no va a git) |
| Plan maestro tecnico | `MASTER_PLAN.md` |
| Guia de la fase actual | `docs/FASE4_GUIA_NAV2.md` |
| Lectures (teoria, abrir en navegador) | `docs/lectures/index.html` |
| Git workflow | `docs/GITHUB_WORKFLOW.md` |
