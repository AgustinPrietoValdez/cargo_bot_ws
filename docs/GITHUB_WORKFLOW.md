# Guía de Workflow con Git y GitHub

> Documento de aprendizaje y referencia interna del proyecto **cargo_bot_ws**.
> Objetivo: trabajar con un flujo Git/GitHub **profesional** aunque seas un solo
> desarrollador, entendiendo el *por qué* de cada convención y teniendo a mano
> los comandos exactos para copiar y pegar.

---

## Índice

1. [Modelo de branching: GitHub Flow](#1-modelo-de-branching-github-flow)
2. [Naming de branches](#2-naming-de-branches)
3. [Conventional Commits](#3-conventional-commits)
4. [La rutina por fase (paso a paso)](#4-la-rutina-por-fase-paso-a-paso)
5. [Estrategia de merge: Squash](#5-estrategia-de-merge-squash)
6. [Pull Requests](#6-pull-requests)
7. [Versionado y Releases](#7-versionado-y-releases)
8. [Gestión del proyecto: Milestones, Issues, Projects](#8-gestión-del-proyecto-milestones-issues-projects)
9. [Regla doc-sync](#9-regla-doc-sync)
10. [Settings del repo](#10-settings-del-repo)
11. [Qué NO se commitea](#11-qué-no-se-commitea)
12. [Cheat sheet](#12-cheat-sheet)

---

## 1. Modelo de branching: GitHub Flow

Adoptamos **GitHub Flow**, no Git Flow.

**Reglas de oro:**

- `main` **siempre anda** y es *deployable*: si alguien clona `main` y hace `colcon build`, compila y el robot arranca.
- **NUNCA** se commitea directo a `main`.
- **Todo** cambio pasa por: *crear branch* → *commits* → *Pull Request (PR)* → *merge*.

### ¿Por qué GitHub Flow y no Git Flow?

**Git Flow** clásico tiene 5 tipos de branches permanentes/semi-permanentes
(`main`, `develop`, `feature/*`, `release/*`, `hotfix/*`). Eso tiene sentido en
equipos grandes con releases versionados pesados y varias versiones soportadas
en paralelo (ej: mantener v1.x mientras desarrollás v2.x).

Para un **proyecto chico y de un solo dev** como este, Git Flow es **sobreingeniería**:

| Problema de Git Flow acá | Por qué GitHub Flow gana |
|---|---|
| Mantener `develop` *y* `main` sincronizados es laburo extra sin beneficio | Una sola rama estable: `main` |
| `release/*` branches asumen ciclos de release formales | Acá releaseamos por *hito de fase*, no por cronograma |
| Más ramas = más merges = más conflictos | Branch corto y efímero por tarea, se borra al mergear |

**GitHub Flow** es: una rama estable (`main`) + ramas cortas y descartables por
cada cosa que hacés. Simple, rápido, y te enseña la disciplina de PRs sin la
burocracia. Es el flujo que usa GitHub internamente y el estándar de facto en
proyectos open source modernos.

### 1b. Granularidad: una branch por TAREA (regla fija, 2026-06-06)

La unidad de branching es la **tarea de la app Plan**, ni más chica ni más grande:

| Nivel | Unidad git | Por qué |
|---|---|---|
| **Subtarea** | Un **commit** (Conventional Commit) dentro de la branch | Una branch+PR por checkbox sería puro overhead; el detalle fino vive en los commits del PR |
| **Tarea** | Una **branch + PR** (squash al completarla) | Tamaño justo: revisable, revertible de a 1 commit en `main`, y mapea 1:1 con el board de Plan |
| **Fase** | Un **tag/release** al cerrar (`v0.X-<tema>`) + **post manual** en LinkedIn (Claude draftea EN/ES, vos publicás) | El hito grande se marca, no se branchea |

- Nombre de branch derivado de la tarea: ej. tarea "Nav2 tuning + waypoints" →
  `feat/fase4-nav2-tuning`.
- Trabajo meta suelto (docs, tooling, configs) que no es una tarea de Plan: branch
  `docs/...`/`chore/...` cortita + PR igual. **Nada directo a `main`** (la protección
  existe para eso; no se bypasea).
- **Review previa (regla 2026-06-06):** antes de commitear o abrir una PR, Claude avisa
  y se revisan los cambios juntos (qué archivos + resumen del diff). Sin OK no hay commit.

---

## 2. Naming de branches

El nombre de la branch arranca con un **prefijo** que dice *qué tipo de trabajo*
es, seguido de una descripción corta en `kebab-case`:

```
<tipo>/<descripción-corta>
```

| Prefijo | Para qué | Ejemplo |
|---|---|---|
| `feat/` | Funcionalidad nueva | `feat/fase4-nav2` |
| `fix/` | Corrección de bug | `fix/ekf-yaw-drift` |
| `docs/` | Solo documentación | `docs/readme` |
| `chore/` | Tareas de mantenimiento, config, deps | `chore/gitignore-isaac` |
| `refactor/` | Reorganizar código sin cambiar comportamiento | `refactor/scripts-setup-diagnostic` |

**¿Por qué prefijos?** Al mirar `git branch` o la lista de branches en GitHub,
de un vistazo sabés qué hay en cada una. Además te obliga a pensar *qué tipo de
cambio* estás haciendo antes de empezar (y a no mezclar un fix con una feature).

> Regla práctica: la descripción es corta pero específica. `feat/fase4-nav2`
> es mejor que `feat/nav` o `feat/cambios`.

---

## 3. Conventional Commits

Los mensajes de commit siguen el estándar **[Conventional Commits](https://www.conventionalcommits.org/)**:

```
tipo(scope): descripción en minúscula, imperativo, sin punto final
```

- **tipo**: la naturaleza del cambio (tabla abajo).
- **scope** (opcional pero recomendado): qué parte del proyecto toca (ej: `slam`, `ekf`, `lidar`, `bringup`, `nav2`).
- **descripción**: imperativo presente ("agrega", "corrige"), no pasado ("agregó").

### Tipos

| Tipo | Cuándo se usa |
|---|---|
| `feat` | Una funcionalidad nueva |
| `fix` | Corrección de un bug |
| `docs` | Cambios solo en documentación |
| `chore` | Config, dependencias, tooling, cosas que no son código de producto |
| `refactor` | Cambio interno que no altera comportamiento observable |
| `test` | Agregar o corregir tests |

### Ejemplos reales del proyecto

```text
feat(slam): integra slam_toolbox y guarda mapa cuarto_v1

fix(ekf): ancla el yaw absoluto del IMU para frenar el drift de heading

fix(lidar): vuelve la orientación del sensor a identidad (el +90X rompía el scan)

chore(bringup): agrega nodo scan_angle_fixer para el off-by-one de beams de Isaac

docs(fase3): documenta el procedimiento IMU + EKF + localization
```

### ¿Por qué sirve tanta ceremonia?

- **Changelog automático**: herramientas pueden generar el changelog leyendo los `feat`/`fix`.
- **`git bisect` más claro**: cuando buscás qué commit rompió algo, mensajes consistentes te orientan.
- **Claridad e historia legible**: `git log --oneline` se lee como una lista de cambios, no como un diario críptico.
- **Disciplina mental**: pensar el `tipo(scope)` te fuerza a hacer commits *atómicos* (un commit = un cambio lógico).

### Trailer `Co-Authored-By`

Cuando un cambio se hizo con asistencia (por ejemplo de un agente de IA), se
agrega al **final** del mensaje, separado por una línea en blanco:

```text
feat(nav2): agrega configuración base de costmaps

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
```

GitHub reconoce el trailer `Co-Authored-By:` y atribuye el commit a múltiples autores.

---

## 4. La rutina por fase (paso a paso)

Esta es la receta exacta para cada bloque de trabajo (una fase, una feature, un fix).
Copiá y adaptá los nombres.

### Paso 1 — Crear la branch desde un `main` actualizado

```bash
git switch main
git pull                       # traé lo último de main
git switch -c feat/fase4-nav2  # crea y se cambia a la branch nueva
```

> `git switch -c` es el equivalente moderno de `git checkout -b`. Más claro: `switch` cambia de rama, `-c` la crea.

### Paso 2 — Trabajar con commits chicos

Hacé commits **atómicos** a medida que avanzás (no un commitón gigante al final):

```bash
git add config/nav2_params.yaml
git commit -m "feat(nav2): agrega parametros base de costmap"

git add launch/nav2.launch.py
git commit -m "feat(nav2): agrega launch file de nav2"
```

> Commits chicos = historia entendible + fácil de revertir una pieza sin tirar todo.

### Paso 3 — Push de la branch al remoto

```bash
git push -u origin feat/fase4-nav2
```

> El `-u` (`--set-upstream`) vincula tu branch local con la remota. La primera
> vez lo ponés; después alcanza con `git push`.

### Paso 4 — Abrir el Pull Request

```bash
gh pr create --fill
```

> `--fill` usa el último commit (o el conjunto) para autocompletar título y cuerpo.
> Si querés escribirlo a mano: `gh pr create` (te abre el editor / prompts).

### Paso 5 — Revisar tu propio diff

Antes de mergear, **leé el diff completo** como si fueras otra persona:

```bash
gh pr diff           # ver el diff en la terminal
gh pr view --web     # abrirlo en el navegador
```

> Este *self-review* es donde cazás el `print()` olvidado, el archivo de build
> que se coló, o el TODO sin terminar.

### Paso 6 — Mergear con squash y borrar la branch

```bash
gh pr merge --squash --delete-branch
```

> Squash colapsa todos los commits del PR en **uno solo** en `main` (ver sección 5).
> `--delete-branch` borra la branch remota; mantené el repo limpio.

### Paso 7 — Volver a `main` y actualizarlo

```bash
git switch main
git pull
git branch -d feat/fase4-nav2   # borra la branch local (ya mergeada)
```

### Paso 8 — Taggear el hito (si la fase quedó completa)

```bash
git tag -a v0.4-nav2 -m "Fase 4: navegacion autonoma con Nav2"
git push origin v0.4-nav2
```

> Ver sección 7 para releases. Taggeás cuando un **hito de fase** queda cerrado, no en cada PR.

---

## 5. Estrategia de merge: SQUASH

Configuramos el repo para **merge solo por squash**. Cuando mergeás un PR, todos
los commits de la branch se aplastan en **un único commit** sobre `main`.

### Comparación de estrategias

| Estrategia | Qué deja en `main` | Pro | Contra |
|---|---|---|---|
| **Merge commit** | Todos los commits de la branch + un commit de merge extra | Conserva el detalle completo | `main` se llena de commits WIP ("fix typo", "wip", "ahora si") + el grafo se vuelve un plato de fideos |
| **Rebase** | Todos los commits de la branch, linealizados | Historia lineal sin merge commits | Reescribe hashes; con commits sucios sigue siendo ruido |
| **Squash** ✅ | **1 commit limpio por PR** | `main` se lee como un changelog: 1 línea = 1 feature/fix; fácil de revertir una fase entera | Se pierde el detalle commit-por-commit de la branch (no importa, ese detalle vive en el PR) |

### ¿Por qué squash para este proyecto?

- En una branch hacés commits chicos y a veces feos ("fix typo", "vuelvo atrás esto").
  Ese ruido **no debería** vivir en `main`.
- Con squash, `main` queda con **1 commit por fase/PR**, perfectamente alineado
  con los hitos del [MASTER_PLAN](../MASTER_PLAN.md).
- Revertir es trivial: `git revert <commit>` deshace una fase entera de un saque.
- El detalle granular no se pierde: queda registrado en el PR en GitHub.

El repo está configurado en **merge-solo-squash** + **auto-delete branch**, así que
no podés equivocarte: la única opción al mergear es squash, y la branch se borra sola.

> Tip: cuando hacés `gh pr merge --squash`, GitHub te deja editar el mensaje
> del commit final. Aprovechá para dejar un buen mensaje Conventional Commit.

---

## 6. Pull Requests

**Sí, usamos PRs aunque trabajes solo.** No es burocracia inútil:

- **Changelog y trazabilidad**: cada PR es una unidad de cambio documentada y enlazable.
- **Self-review del diff**: te obliga a mirar tu propio trabajo con ojo crítico *antes* de que toque `main`. Cazás un montón de errores ahí.
- **Futura CI**: el día que agregues GitHub Actions (build + tests automáticos),
  el PR es el punto natural donde corren los checks antes de mergear. Empezar con
  PRs ahora hace que ese paso sea gratis después.
- **Checklist**: el PR te recuerda los pasos que no se ven en el código (¿actualicé los docs? ¿taggié el hito?).

### Template de PR

Hay un **template de PR** en `.github/` (`.github/pull_request_template.md`).
GitHub lo carga automáticamente cuando abrís un PR, con secciones tipo:

- **Qué hace** este PR.
- **Cómo se probó** (comandos, topics verificados, `ros2 topic hz`, etc.).
- **Checklist** (docs sincronizados, tag si corresponde, sin artefactos de build).

> No borres las secciones del template: completalas. Te fuerzan a no olvidarte de nada.

---

## 7. Versionado y Releases

Marcamos cada **hito de fase** con un **tag** estilo *semver-ish*:

```
v0.<fase>-<nombre>
```

| Tag | Hito |
|---|---|
| `v0.3-slam` | Fase 3: SLAM completa, mapa guardado |
| `v0.4-nav2` | Fase 4: navegación autónoma con Nav2 |
| `v0.5-...` | (próximas fases) |

> Usamos `v0.x` porque el proyecto está pre-1.0 (todavía en construcción).
> Llegaremos a `v1.0` cuando el robot haga el caso de uso completo de forma estable.

El tag **`v0.3-slam` ya existe** (marca el cierre de la Fase 3).

### Crear un tag anotado

Un tag **anotado** (`-a`) guarda autor, fecha y mensaje (a diferencia de un tag
liviano, que es solo un puntero). Siempre usá anotados para hitos:

```bash
git switch main
git pull
git tag -a v0.4-nav2 -m "Fase 4: navegacion autonoma con Nav2"
git push origin v0.4-nav2
```

### Crear un GitHub Release con notas

Un *Release* en GitHub es un tag + notas + (opcional) binarios adjuntos. Sirve
como punto de referencia citable del estado del proyecto en ese hito:

```bash
gh release create v0.4-nav2 \
  --title "v0.4 — Navegación autónoma (Nav2)" \
  --notes "Fase 4 completa: Nav2 navegando sobre el mapa de la Fase 3.

- Costmaps configurados (global + local)
- AMCL localizando contra cuarto_v1
- Goals por RViz funcionando"
```

> Las notas del release son tu changelog legible por humanos del hito. Resumí
> lo que cambió desde el tag anterior.

---

## 8. Gestión del proyecto: Milestones, Issues, Projects

El plan vive en el código y en GitHub, sincronizados:

| Concepto GitHub | Qué representa en el proyecto |
|---|---|
| **Milestone** | Una **fase** completa (ej: "Fase 4 — Nav2") |
| **Issue** | Una **tarea** dentro de una fase (ej: "Configurar costmaps", "Tunear AMCL") |
| **Projects (board)** | (opcional) Roadmap visible tipo Kanban: To do / Doing / Done |

### Cómo se relaciona con el MASTER_PLAN

El [`MASTER_PLAN.md`](../MASTER_PLAN.md) es la **única fuente de verdad operativa**
del proyecto (qué fases hay, en qué orden, qué falta). GitHub es el reflejo
*ejecutable* de ese plan:

- Cada **fase** del MASTER_PLAN → un **Milestone**.
- Cada **paso/tarea** de una fase → un **Issue**, asignado a su Milestone.
- Al cerrar un PR que completa una tarea, se cierra el Issue (`Closes #N` en el cuerpo del PR lo hace automático).
- Cuando todos los Issues de un Milestone están cerrados → la fase está completa → taggeás el hito (sección 7).

```bash
# crear un issue desde la terminal
gh issue create --title "Configurar costmaps de Nav2" --milestone "Fase 4 — Nav2"

# cerrar issues automáticamente desde un PR: poné esto en el cuerpo del PR
# Closes #12
```

> El board de Projects es **opcional**: útil para ver el roadmap de un vistazo,
> pero no es obligatorio. Los Milestones + Issues alcanzan para la trazabilidad.

---

## 9. Regla doc-sync

> **Toda decisión que cambie el plan se refleja en `MASTER_PLAN.md` +
> `docs/FASE*_GUIA_*.md` en el MISMO PR que la implementa.**

¿Por qué en el mismo PR y no "después"?

- Los docs son la **fuente de verdad operativa** que se lee como guía. Si el
  código y los docs divergen, los docs mienten y dejás de confiar en ellos.
- Si lo dejás "para después", *nunca* pasa. La doc queda stale (ver el caso de
  `FASE2_GUIA_ISAAC_SIM.md` que quedó con patrones viejos de Isaac 4.x).
- El PR es la unidad atómica de cambio: código + su documentación viajan juntos
  y se revisan juntos.

**En la práctica:** si en un PR cambiás cómo funciona el EKF, ese mismo PR
actualiza la sección del EKF en la guía de fase correspondiente y, si cambió el
plan, el MASTER_PLAN.

---

## 10. Settings del repo

Configuración del repositorio en GitHub y el porqué de cada decisión:

| Setting | Valor | Por qué |
|---|---|---|
| **Default branch** | `main` | La rama estable y deployable; todo nace y vuelve acá |
| **Branch protection en `main`** | Obliga PR; **sin** requerir approvals externos | No podés pushear directo a `main` (te protege de vos mismo). No exigimos approval de terceros porque sos un solo dev — sería un deadlock |
| **Merge button** | Solo **squash** | `main` queda con 1 commit limpio por PR (sección 5) |
| **Auto-delete branches** | Activado | Al mergear, la branch remota se borra sola; el repo no se llena de ramas muertas |
| **LICENSE** | MIT | Licencia permisiva estándar; deja claro que otros pueden usar/forkear el código |
| **Description + Topics** | Completos (ej: `ros2`, `isaac-sim`, `slam`, `nav2`, `robotics`) | Hace el repo descubrible y deja claro de qué se trata de un vistazo |

> La **branch protection** es la pieza clave que hace cumplir GitHub Flow: aunque
> te tiente hacer `git push` directo a `main` un viernes a la noche, GitHub te lo
> rechaza y te obliga a pasar por un PR.

---

## 11. Qué NO se commitea

El `.gitignore` cubre todo lo que es **generado** o **local**. La regla de oro:
**nunca commitees artefactos de build ni cosas reproducibles**.

Lo que queda fuera del repo:

- **Artefactos de colcon**: `build/`, `install/`, `log/` — se regeneran con `colcon build`. Commitearlos infla el repo y genera conflictos infinitos.
- **Config de IDE**: `.vscode/`, `.idea/` — son tuyas, no del proyecto.
- **Logs**: archivos `*.log`, dumps de diagnóstico.
- **PDFs de TF**: el `frames.pdf` / `frames.gv` que genera `ros2 run tf2_tools view_frames`.
- **Dumps de diagnóstico** y salidas temporales de scripts.
- **Capturas sueltas**: screenshots `.png` random que tirás en la raíz (las imágenes de doc van en una carpeta versionada a propósito, no sueltas).

> ¿Por qué tan estricto? Porque `main` debe ser **fuente** (código + config +
> docs), no **derivados**. Cualquiera clona y hace `colcon build` para obtener
> los binarios. Si un artefacto de build llega a un PR, el self-review (Paso 5)
> es donde lo cazás antes de que ensucie `main`.

Si por error trackeaste algo que no debías:

```bash
git rm -r --cached build install log   # lo saca del índice sin borrarlo del disco
# luego asegurate de que esté en .gitignore y commiteá
```

---

## 12. Cheat sheet

Comandos más usados, listos para copiar:

```bash
# --- Empezar una tarea ---
git switch main && git pull
git switch -c feat/faseN-descripcion

# --- Trabajar ---
git add <archivos>
git commit -m "feat(scope): descripcion imperativa"
git status                       # ver qué cambió
git diff                         # ver el diff sin stagear

# --- Subir y abrir PR ---
git push -u origin feat/faseN-descripcion   # primera vez (con -u)
git push                                     # siguientes veces
gh pr create --fill

# --- Revisar antes de mergear ---
gh pr diff
gh pr view --web

# --- Mergear (squash + borra branch) ---
gh pr merge --squash --delete-branch

# --- Volver a main limpio ---
git switch main && git pull
git branch -d feat/faseN-descripcion         # borra la branch local

# --- Taggear un hito de fase ---
git tag -a v0.N-nombre -m "Fase N: descripcion"
git push origin v0.N-nombre
gh release create v0.N-nombre --title "v0.N — Nombre" --notes "..."

# --- Issues / Milestones ---
gh issue create --title "Tarea X" --milestone "Fase N — Nombre"
gh issue list
# (en el cuerpo del PR) Closes #N   -> cierra el issue al mergear

# --- Emergencia: saqué algo del tracking ---
git rm -r --cached build install log
```

### Convención de mensajes (recordatorio rápido)

```
tipo(scope): descripcion en minuscula, imperativo, sin punto

[cuerpo opcional explicando el por qué]

Co-Authored-By: Nombre <email>   <- solo si hubo asistencia
```

Tipos: `feat` · `fix` · `docs` · `chore` · `refactor` · `test`

---

*Esta guía es parte de la documentación operativa del proyecto. Si cambiás el
workflow, actualizá este archivo en el mismo PR (regla doc-sync, sección 9).*
