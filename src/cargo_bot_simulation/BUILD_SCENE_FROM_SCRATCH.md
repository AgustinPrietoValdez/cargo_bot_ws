# Build the cargo_bot Isaac Sim 5.1 scene from scratch

> **Read this if you're starting fresh.** The previous `scenes/scene.usda`
> accumulated dead Replicator render products, a double articulation root,
> and a broken IsaacCreateRenderProduct OG node. This guide rebuilds the
> scene cleanly on Isaac Sim 5.1.0-rc.19 with the URDF at
> `C:\Users\agusp\cargo_bot_ws\cargo_bot_isaac.urdf`.
>
> **Target output:** `src/cargo_bot_simulation/scenes/scene_v2.usda`
> (USDA text, not binary, NOT overwriting `scene.usda`).

---

## Why we are NOT trying to fix `scene.usda`

The previous scene picked up at least 5 defects that we cannot strip safely:

1. Three orphan `Replicator_NN` render products with `LdrColor` AOV bound to
   an `OmniLidar` (RTX sensor renderer silently emits zeros).
2. A duplicate `IsaacCreateRenderProduct` OG node that spawns a new
   Replicator on every Play.
3. `PhysicsArticulationRootAPI` applied to both `base_footprint` AND an
   implicit `root_joint` -- PhysX tensors plugin breaks on the second one.
4. The URDF Importer flattened the link hierarchy
   (`/cargo_bot/lidar_link` instead of
   `/cargo_bot/base_footprint/base_link/lidar_link`). Some OG paths in the
   current scene assume the nested form.
5. Stale `no_delete=true` metadata on the bad RPs so `DeletePrims` silently
   no-ops, even when run inside an `EditContext` on the right layer.

Building from scratch in USDA + Python takes ~20 minutes and is auditable
diff-by-diff. Trying to repair `scene.usda` has cost more than a day
already.

---

## Pre-requisites (do these BEFORE Step 1)

```text
[ ] Close every running Isaac Sim window (Task Manager: kill kit.exe if any).
[ ] In WSL, start the Discovery Server in a dedicated terminal:
        bash /mnt/c/Users/agusp/cargo_bot_ws/config/start_discovery_server.sh
    Leave it running for the whole session.
[ ] cargo_bot_isaac.urdf must exist:
        dir C:\Users\agusp\cargo_bot_ws\cargo_bot_isaac.urdf
    (If missing, regenerate per docs/FASE2_GUIA_ISAAC_SIM.md section 2.1.)
[ ] Free at least 4 GB of VRAM (close Chrome/games).
```

---

## Step 1 -- Launch Isaac GUI cleanly

1. Double-click `C:\Users\agusp\cargo_bot_ws\config\launch_isaac_ros.cmd`.
2. Wait for the splash, then the empty Isaac Sim window.
3. **Verify env** -- open `Window -> Script Editor`, paste and Run:
   ```python
   import os
   for k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"):
       print(k, "=", os.environ.get(k))
   ```
   Expected:
   ```
   ROS_DOMAIN_ID = 1
   RMW_IMPLEMENTATION = rmw_fastrtps_cpp
   FASTRTPS_DEFAULT_PROFILES_FILE = C:\Users\agusp\cargo_bot_ws\config\fastdds_isaac.xml
   ```
4. Confirm the ROS 2 bridge is alive -- in the same Script Editor:
   ```python
   from isaacsim.core.utils.extensions import enable_extension
   enable_extension("isaacsim.ros2.bridge")
   enable_extension("isaacsim.sensors.rtx")
   print("bridge + RTX sensors OK")
   ```

**Success** -- you see `bridge + RTX sensors OK` and no red error lines in
the bottom Console panel.

**Trap** -- if `ROS_DOMAIN_ID` is empty, the `.cmd` did not set env correctly.
Close Isaac and re-launch from the `.cmd` (NOT from Start Menu).

---

## Step 2 -- Import the URDF

1. `File -> Import...` (NOT `File -> Open`).
2. Navigate to and select `C:\Users\agusp\cargo_bot_ws\cargo_bot_isaac.urdf`.
3. The URDF Importer dialog opens. Set fields exactly as below
   (ref: <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html>):

   | Field                          | Value                                  |
   | ------------------------------ | -------------------------------------- |
   | Output Directory               | `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scenes\imported_urdf` |
   | Merge Fixed Joints             | **OFF** (we need `lidar_link` as a discoverable Xform) |
   | Fix Base Link                  | **OFF** (mobile base, not a fixed arm) |
   | Self-Collision                 | **OFF**                                |
   | Replace Cylinders With Capsules| **OFF**                                |
   | Default Drive Strength         | `0` (we set per-joint drives in Step 4)|
   | Default Position Drive Damping | `0`                                    |
   | Distance Scale                 | `1.0`                                  |
   | Up Vector                      | `Z`                                    |
   | Joint Drive Type               | **Velocity**                           |
   | Joint Drive Strength           | `1e3` (damping for the velocity drives we want on wheels) |
   | Convex Decomposition           | **ON** (collision meshes)              |
   | Collision From Visuals         | **OFF**                                |
   | Create Physics Scene           | **OFF** (we add it manually so it has a known path) |
   | Make Default Prim              | **ON**                                 |

4. Click `Import`. Wait ~10-30 s. The robot appears under
   `/cargo_bot` in the Stage panel.

5. **Verify hierarchy** in the Stage panel -- expect either of these two shapes
   (5.1.0-rc.19 may flatten; both are workable -- our scripts reference
   `/cargo_bot/lidar_link` either way):

   ```
   /cargo_bot                       (Xform, articulation root candidate)
     base_footprint                 (Xform)  or  base_link
     base_link
     lidar_link
     left_wheel_link
     right_wheel_link
     caster_wheel_link
     imu_link
     <joints>                       (root_joint, left_wheel_joint, ...)
   ```

> **Trap (flattened hierarchy):** In 5.1.0-rc.19 the URDF Importer flattens
> all links to direct children of `/cargo_bot`. That is OK -- our scripts
> (`add_lidar.py`, `publish_lidar.py`) and the OG paths in this guide all use
> `/cargo_bot/lidar_link` (flat). Do **not** try to manually re-nest the
> hierarchy: it breaks the joint relationships.

> **Trap (URDF Importer doubles articulation root):** the importer applies
> `PhysicsArticulationRootAPI` to `/cargo_bot` (the default prim) AND to an
> implicit `root_joint` underneath. PhysX builds its tensor pattern as
> `<robotPath>/root_joint` and crashes. Fix it in Step 3.

> **Trap (lidar.stl import):** the visual mesh `meshes/visual/lidar.stl` was
> exported by Fusion 360 with a non-standard binary header (`STLB ATF
> 15.8.0.0 ...`) and the URDF Importer may silently drop it. This is
> non-blocking -- the lidar still functions as an OmniLidar sensor; you just
> won't see the puck mesh in the viewport.

---

## Step 3 -- Verify and fix articulation root

We want **exactly one** Articulation Root.  In Isaac Sim 5.1.0-rc.19 the
URDF Importer applies it to `/cargo_bot/base_footprint` (the URDF root link).
Confirm there is only ONE root and that it's there.

1. Run the **Confirm via Script Editor** snippet at the end of this section.
   Expected output: `Articulation roots: ['/cargo_bot/base_footprint']`.
2. If the snippet returns more than one path (e.g. ALSO `/cargo_bot` or
   `/cargo_bot/root_joint`), the URDF Importer duplicated it. Remove the
   extras:
   - Select each extra prim in the Stage panel.
   - In the Property panel under "Applied Schemas" find
     `PhysicsArticulationRootAPI`.
   - Right-click on the schema row -> **Remove Articulation Root API**.
   - Re-run the snippet -- should now show only `/cargo_bot/base_footprint`.

**Confirm via Script Editor** (paste, Run):
```python
import omni.usd
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()
roots = []
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        roots.append(str(p.GetPath()))
print("Articulation roots:", roots)
```
Expected output: `Articulation roots: ['/cargo_bot/base_footprint']`.
Anything else (extra paths, or a different single path) = remove the extras
and re-run the snippet.

> **Trap:** If you see 2+ roots, PhysX still works partially but
> `Articulation Controller` (Step 6) will pick the wrong one and you'll
> spend an hour wondering why the wheels don't spin. ALWAYS confirm exactly
> one root before continuing.

---

## Step 4 -- Configure wheel joint drives

The Differential Controller (Step 6) writes velocity targets to
`left_wheel_joint` and `right_wheel_joint`. These joints need velocity
drives configured correctly.

For each of `left_wheel_joint` and `right_wheel_joint`:

1. Select the joint in the Stage panel.
2. In the Property panel, find **Drive** under the joint's properties.
   - If absent: click `+ Add` -> `Physics` -> `Angular Drive`.
3. Set the drive fields exactly:

   | Field            | Value      |
   | ---------------- | ---------- |
   | Type             | `angular`  |
   | Target Type      | `velocity` |
   | Target Position  | `0`        |
   | Target Velocity  | `0`        |
   | Damping          | `1000`     |
   | Stiffness        | `0`        |
   | Max Force        | `10000`    |

   Stiffness MUST be 0 for a velocity drive -- a non-zero stiffness adds a
   position spring that fights the velocity command.

4. **Caster joint (`caster_wheel_joint`)**: leave it as a `FixedJoint` with
   NO drive. The caster is a passive ball wheel.
5. **Lidar / IMU joints**: `FixedJoint`, no drive, no changes needed.

**Confirm via Script Editor**:
```python
import omni.usd
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()
for jname in ("left_wheel_joint", "right_wheel_joint"):
    for p in stage.Traverse():
        if p.GetName() == jname:
            d = UsdPhysics.DriveAPI.Get(p, "angular")
            print(jname,
                  "damping=", d.GetDampingAttr().Get(),
                  "stiffness=", d.GetStiffnessAttr().Get(),
                  "max_force=", d.GetMaxForceAttr().Get())
            break
```
Expected: `damping=1000 stiffness=0 max_force=10000` for both.

> **Trap:** if the URDF defines `<dynamics>` with non-zero damping, the
> importer may already have populated drives with junk values. Always
> overwrite with the table above.

---

## Step 5 -- Add the RTX Lidar via Python (NOT the menu)

We use the Python command `IsaacSensorCreateRtxLidar`. The Slamtec
`RPLIDAR_S2E` config ships with Isaac at
`C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\data\lidar_configs\SLAMTEC\RPLIDAR_S2E.json`
and is selected by the config string `"Slamtec/RPLIDAR_S2E"`.

> **Trap (do NOT use the GUI menu):** the menu path
> `Create -> Sensors -> RTX Lidar -> Slamtec -> RPLIDAR S2E`, when invoked
> with `/cargo_bot/lidar_link` selected as parent, wraps the new prim in an
> extra Xform named after the parent. You end up with
> `/cargo_bot/lidar_link/cargo_bot/RPLIDAR_S2E/RPLidar_S2E` instead of a
> clean leaf path. The OG references in Step 8 then point at the wrong
> path.

1. Open the Script Editor: `Window -> Script Editor`.
2. `File -> Open ...` -> `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\add_lidar.py`.
3. Click **Run** (or Ctrl-Enter).
4. Expected console output:
   ```
   [add_lidar] step 1 stage acquired
   [add_lidar] step 2 no pre-existing lidar prim (clean)
   [add_lidar] step 3 parent /cargo_bot/lidar_link OK
   [add_lidar] step 4 created OmniLidar at /cargo_bot/lidar_link/lidar_sensor  type=OmniLidar
   [add_lidar] DONE.  Now press Play and run publish_lidar.py.
   ```
5. **Verify in Stage panel:** `/cargo_bot/lidar_link/lidar_sensor` is
   present with type `OmniLidar`. Expand its properties -- you should see
   the applied schema `OmniSensorGenericLidarCoreAPI`.

The script is **idempotent**: re-running deletes the previous
`/cargo_bot/lidar_link/lidar_sensor` first, then creates a fresh one.

---

## Step 6 -- Set up the cmd_vel control chain (manual)

The shortcut `Tools -> Robotics -> ROS 2 OmniGraphs` does NOT include a
`cmd_vel` subscribe pattern, so this is manual.

1. `Window -> Graph Editors -> Action Graph`.
2. `New Action Graph` -> name it `cmd_vel_graph`. It is created at
   `/cargo_bot/cmd_vel_graph` (default).
3. Add these nodes one by one (search in the node browser by display name):

   | Display Name             | Node type (search keyword)        |
   | ------------------------ | --------------------------------- |
   | On Playback Tick         | `omni.graph.action.OnPlaybackTick`|
   | ROS2 Subscribe Twist     | `ROS2 Subscribe Twist`            |
   | Break 3-Vector           | `Break 3-Vector`                  |
   | Break 3-Vector           | `Break 3-Vector` (add a SECOND)   |
   | Differential Controller  | `Differential Controller`         |
   | Articulation Controller  | `Articulation Controller`         |

4. **Wire them** (drag from output pin to input pin):

   ```
   OnPlaybackTick.tick  ────────────► ROS2SubscribeTwist.execIn
   ROS2SubscribeTwist.execOut ──────► DifferentialController.execIn
                              \───►   ArticulationController.execIn

   ROS2SubscribeTwist.linearVelocity  ─► BreakVector_linear.tuple
   BreakVector_linear.x ──────────────► DifferentialController.linearVelocity

   ROS2SubscribeTwist.angularVelocity ─► BreakVector_angular.tuple
   BreakVector_angular.z ─────────────► DifferentialController.angularVelocity

   DifferentialController.velocityCommand ─► ArticulationController.velocityCommand
   ```

5. **Set node properties:**

   `ROS2 Subscribe Twist`:
   | Property   | Value      |
   | ---------- | ---------- |
   | topicName  | `/cmd_vel` |
   | qosProfile | `default`  |

   `Differential Controller`:
   | Property        | Value  |
   | --------------- | ------ |
   | wheelDistance   | `0.29` |
   | wheelRadius     | `0.10` |
   | maxWheelSpeed   | `10.0` |
   | maxLinearSpeed  | `1.0`  |
   | maxAngularSpeed | `3.0`  |

   (Read the actual `wheel_separation` and `wheel_radius` from
   `src/cargo_bot_description/urdf/cargo_bot.urdf.xacro` if you tweaked
   them since this guide was written. The above are the project defaults.)

   `Articulation Controller`:
   | Property    | Value                                            |
   | ----------- | ------------------------------------------------ |
   | usePath     | `true`                                           |
   | robotPath   | `/cargo_bot/base_footprint`                      |
   | jointNames  | `["left_wheel_joint", "right_wheel_joint"]`     |

> **Trap:** `robotPath` MUST be the prim that has Articulation Root (set in
> Step 3). If you mistakenly point it at `/cargo_bot/root_joint`, PhysX
> builds an invalid pattern and silently emits zero velocity.

> **Trap:** the Differential Controller emits a 2-element array. The
> Articulation Controller `velocityCommand` input expects a length-2 array
> in the SAME order as `jointNames`. Keep the order
> `[left_wheel_joint, right_wheel_joint]` consistent on both sides.

6. **Test cmd_vel only (before adding sensors):**
   - Press Play.
   - In WSL: `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' -r 10`
   - The robot should crawl forward in the viewport. Ctrl-C the pub.
   - If it doesn't move: check the Articulation Controller's `robotPath`,
     the joint drive damping (Step 4), and confirm Articulation Root
     (Step 3) is on exactly `/cargo_bot/base_footprint`.

---

## Step 7 -- Set up Odom / TF / Clock via the shortcut

These shortcuts are safe (they do NOT spawn Replicator render products).
Run them with `/cargo_bot` selected in the Stage panel.

1. **Clock**:
   `Tools -> Robotics -> ROS 2 OmniGraphs -> Clock`.
   - Graph path: `/cargo_bot/ros_clock_graph` (default).
   - Topic: `/clock` (default). Leave as-is.

2. **TF Publisher**:
   `Tools -> Robotics -> ROS 2 OmniGraphs -> TF Publisher`.
   - In the dialog, set:
     - **Target Prims:** `/cargo_bot`
     - **Parent Prim:** leave empty (the world acts as the root frame)
     - **Topic Name:** `/tf` (default)
   - The shortcut creates `/cargo_bot/ros_tf_graph`.

3. **Odometry**:
   `Tools -> Robotics -> ROS 2 OmniGraphs -> Odometry`.
   - In the dialog, set:
     - **Chassis Prim:** `/cargo_bot/base_footprint` (the articulation root)
     - **Topic Name:** `/odom` (default)
     - **Frame ID:** `odom`
     - **Child Frame ID:** `base_footprint` (or `base_link` if your URDF
       flattened away `base_footprint` -- check the Stage panel and pick
       the one that's a direct child of `/cargo_bot`)
   - The shortcut creates `/cargo_bot/ros_odometry_graph`.

> **Trap (the lidar shortcut):** Do NOT use
> `Tools -> Robotics -> ROS 2 OmniGraphs -> RTX Lidar`. Its
> `IsaacCreateRenderProduct` OG node calls
> `rep.create.render_product(camera_path, force_new=True)` inside its
> `compute()` and spawns a fresh Replicator on every Play. After the third
> Play you have three render products with `LdrColor` AOV bound to the
> OmniLidar and the RTX-Sensor renderer silently emits zeros. We use
> `publish_lidar.py` in Step 8 instead.

---

## Step 8 -- Publish the lidar via Python

1. Press **Play** in the viewport (timeline must be running for the writer
   to spin up its hydra rendergraph).
2. `Window -> Script Editor` -> `File -> Open ...` ->
   `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\publish_lidar.py`.
3. Click **Run**.
4. Expected console output:
   ```
   [publish_lidar] step 0 cleared previous keepalive refs           (only on re-run)
   [publish_lidar] step 1 lidar prim OK at /cargo_bot/lidar_link/lidar_sensor
   [publish_lidar] step 2 render product created at /Render/.../CargoBotLidar_RP
   [publish_lidar] step 3 orderedVars clean: ['GenericModelOutput', 'RtxSensorMetadata']
   [publish_lidar] step 4 RtxLidarROS2PublishLaserScan attached  topic=/scan_py  frame=lidar_link
   [publish_lidar] step 5 debug-draw attached (optional)
   [publish_lidar] step 6 stashed keepalive refs in builtins._cargo_bot_lidar_keepalive
   [publish_lidar] DONE.  /scan_py should be live in WSL within ~1 second.
   ```

> **Trap:** the writer references must stay alive for the duration of the
> session. The script stashes them in `builtins._cargo_bot_lidar_keepalive`
> so Python GC does not collect them between Script Editor invocations. Do
> NOT delete that attribute.

---

## Step 9 -- Verify everything works

In WSL, in a terminal sourced with `config/source_ros_wsl.sh`:

```bash
ros2 topic list
# Expected (order may vary):
#   /clock
#   /cmd_vel
#   /odom
#   /scan_py
#   /tf
#   /tf_static
#   /parameter_events
#   /rosout

ros2 topic hz /scan_py            # expected ~10 Hz (S2E scanRateBaseHz=10)
ros2 topic hz /clock              # expected ~60 Hz (matches physics_dt)
ros2 topic hz /odom               # expected ~60 Hz

ros2 topic echo /scan_py --once
# Expected: frame_id="lidar_link", ranges array length ~1066

ros2 topic echo /tf --once
# Expected: parent base_footprint -> child base_link etc.
```

**Drive the robot:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false
```
- `i` should drive it forward in the Isaac viewport.
- `,` (period below `i`) is reverse.
- `j` / `l` should turn it.

**In Isaac viewport (visual check):**
- Lidar rays should be visible as a ring of dots (debug draw from Step 8).
- The wheels should rotate (cyan motion blur when teleop sends commands).
- The robot should NOT sink into the floor or float -- if it does,
  re-check Step 3 (articulation root) and Step 4 (drive damping not zero).

---

## Step 10 -- Save the scene

1. `File -> Save As...`
2. Path: `C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scenes\scene_v2.usda`
3. Format dropdown: select **USDA** (text). NOT `.usd` binary.
4. Click Save.

> **Trap:** Isaac sometimes defaults the Save As dialog to `.usd` binary
> with no format selector visible. If the filename input shows `.usd`,
> manually type the `.usda` extension. Verify after save:
> ```powershell
> Get-Content C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scenes\scene_v2.usda -TotalCount 2
> ```
> First line MUST be `#usda 1.0`. If it's binary garbage, you saved as
> `.usd` -- delete it, save again with the right extension.

> **Trap (Save-As re-introduces Replicators):** if any pre-existing prim in
> the stage has `no_delete=true` (the old `scene.usda` had three), Isaac
> may copy them into the new file on save. Verify after save:
> ```powershell
> Select-String -Path scene_v2.usda -Pattern "Replicator" -SimpleMatch
> ```
> Expected: zero matches (we never spawned a Replicator named that way --
> our render product is named `CargoBotLidar_RP` and lives in the session
> layer, which is NOT saved to disk).

---

## Re-launching after a clean shutdown

Once `scene_v2.usda` exists, the daily workflow is:

1. Start Discovery Server in WSL.
2. Launch Isaac via `launch_isaac_ros.cmd`.
3. `File -> Open` -> `scene_v2.usda`.
4. Re-run `add_lidar.py` only if the lidar prim is missing (rare -- the
   USDA file does save the OmniLidar). Usually skip.
5. Press Play.
6. Run `publish_lidar.py` once per session (the render-product + writer
   bindings live in the session layer, which is fresh on every open).
7. Verify with `ros2 topic hz /scan_py` from WSL.

The two Python scripts are **idempotent** -- re-running them is safe.

---

## Source references

- Isaac Sim 5.1 URDF Importer:
  <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html>
- Isaac Sim 5.1 RTX Lidar Sensor:
  <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx_lidar.html>
- Working bundled example (canonical pattern):
  `C:\isaacsim_51_ga\standalone_examples\api\isaacsim.ros2.bridge\rtx_lidar.py`
- `IsaacSensorCreateRtxLidar` command source (config-matching logic):
  `C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\isaacsim\sensors\rtx\impl\commands.py:237`
- Annotator-attach reference pattern:
  `C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\isaacsim\sensors\rtx\tests\test_annotators.py:255-260`
- Slamtec RPLIDAR S2E config JSON (the config string `"Slamtec/RPLIDAR_S2E"`
  resolves to this file):
  `C:\isaacsim_51_ga\exts\isaacsim.sensors.rtx\data\lidar_configs\SLAMTEC\RPLIDAR_S2E.json`
- Project Phase 2 background (4.x style -- this guide supersedes it for
  Isaac 5.1):
  `docs/FASE2_GUIA_ISAAC_SIM.md`
