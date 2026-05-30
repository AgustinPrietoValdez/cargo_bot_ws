# Getting Started with cargo_bot

A hands-on guide to set up, build, and run the **cargo_bot** simulation stack end to end.

> New here? Read the **[README.md](README.md)** for the project overview, architecture diagrams, and phase status first. The full development plan lives in **[docs/MASTER_PLAN.md](docs/MASTER_PLAN.md)**.

---

## 1. Prerequisites

The project is **sim-first**. Isaac Sim runs natively on **Windows**, and the ROS 2 stack runs in **WSL2**. You need both.

### Windows side
- **Windows 10/11** with a CUDA-capable NVIDIA GPU.
  - Reference dev rig: RTX 4060 Laptop (8 GB VRAM), 16 GB RAM / 16 threads.
- **NVIDIA Isaac Sim 5.1**.

### WSL2 side
- **WSL2 with Ubuntu 22.04**.
- **ROS 2 Humble** (`ros-humble-desktop`).
- Project apt dependencies:

```bash
sudo apt update
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard \
  ros-humble-nav2-map-server \
  python3-colcon-common-extensions
```

> `slam-toolbox` provides 2D SLAM, `robot-localization` provides the EKF, `twist-mux` arbitrates velocity command sources, `teleop-twist-keyboard` is used for manual driving, and `nav2-map-server` provides the map saver. (Nav2 itself arrives with Phase 4.)

---

## 2. Workspace setup

Clone the repository and build the colcon workspace from WSL2.

```bash
# Clone (the dev rig keeps it on the Windows filesystem so Isaac can read scenes)
cd /mnt/c/Users/agusp
git clone https://github.com/AgustinPrietoValdez/cargo_bot_ws.git
cd cargo_bot_ws

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the install space
source install/setup.bash
```

Re-run `source install/setup.bash` in every new shell (or use the helper in step 3).

---

## 3. The DDS Discovery Server and ROS_DOMAIN_ID

Isaac Sim (Windows) and ROS 2 (WSL2) are two separate hosts on the DDS network. To make them discover each other reliably across the Windows ↔ WSL2 boundary, the project uses a **Fast DDS Discovery Server**:

- **WSL2 = SERVER** — runs the discovery server process.
- **Isaac Sim = CLIENT** — points at the server's address.
- Middleware is **`rmw_fastrtps_cpp`** on both sides.
- Both sides must share **`ROS_DOMAIN_ID=1`** and use **`use_sim_time=true`** (the simulation clock is published by Isaac on `/clock`).

The helper script `config/source_ros_wsl.sh` sets the WSL2 environment (RMW implementation, domain id, discovery-server config) for you. The Windows launcher `config/launch_all.cmd` starts the discovery server in WSL2 and brings up Isaac Sim together; it also patches Isaac's discovery-server client address at each launch.

```bash
# In WSL2, source the project environment in each new shell:
cd /mnt/c/Users/agusp/cargo_bot_ws
source config/source_ros_wsl.sh
source install/setup.bash
```

If `ROS_DOMAIN_ID` or the RMW implementation differs between the two sides, **no topics will appear** — see Troubleshooting.

---

## 4. Boot sequence

Follow these five steps in order.

### Step 1 — Start the Discovery Server + Isaac Sim (Windows)
Run the launcher from a Windows terminal (or double-click it):

```bat
config\launch_all.cmd
```

This starts the Fast DDS Discovery Server inside WSL2 and launches Isaac Sim together.

### Step 2 — Open the scene and Play (Isaac Sim)
In Isaac Sim, open the scene **`scene_v4.usda`** (under `src/cargo_bot_simulation`), then press **▶ Play**. Playing the scene starts publishing sensor data and the simulation clock, and starts listening for velocity commands.

### Step 3 — Source the ROS 2 environment (WSL2)
```bash
cd /mnt/c/Users/agusp/cargo_bot_ws
source config/source_ros_wsl.sh
source install/setup.bash
```

### Step 4 — Verify the bridge (WSL2)
```bash
ros2 topic list
```
You should see at least:
```
/scan
/odom
/imu/data
/clock
/cmd_vel
```
Quick sanity checks:
```bash
ros2 topic hz /clock           # clock ticking → Isaac is Playing
ros2 topic echo /odom --once   # odometry flowing
ros2 topic echo /imu/data --once
```

### Step 5 — Launch SLAM / navigation (WSL2)
```bash
ros2 launch cargo_bot_bringup slam.launch.py
```
This brings up the localization + mapping pipeline:
`scan_angle_fixer` (`/scan` → `/scan_fixed`) → `robot_localization` EKF (`/odom` + `/imu/data` → `/odometry/filtered` and the `odom → base_footprint` TF) → `slam_toolbox` (consuming `/scan_fixed`) → `/map` and the `map → odom` TF.

---

## 5. Driving the robot (teleop)

With the scene Playing and your ROS 2 environment sourced, drive manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This publishes `geometry_msgs/Twist` on `/cmd_vel`, which Isaac Sim consumes to move the robot. Positive `linear.x` drives the robot forward. Keep the teleop terminal focused for keypresses to register.

---

## 6. Running SLAM and saving a map

1. Boot everything (Section 4) and launch SLAM (`slam.launch.py`).
2. Open **RViz** to watch the map build (add the `/map` display and the TF tree).
3. Drive the robot slowly with teleop (Section 5) to cover the room; the map grows as new geometry is observed on `/scan_fixed`.
4. When the map looks complete, save it with the slam_toolbox / map saver:

```bash
ros2 run nav2_map_server map_saver_cli -f src/cargo_bot_navigation/maps/my_map
```

This writes `my_map.pgm` + `my_map.yaml`. The reference room map is already saved as **`src/cargo_bot_navigation/maps/cuarto_v1.{pgm,yaml}`** (a ~5.0 × 4.9 m room).

> **Important:** everything downstream of the lidar must consume **`/scan_fixed`**, not `/scan`. The `scan_angle_fixer` node corrects an Isaac RTX-lidar beam-count off-by-one that otherwise causes slam_toolbox / Karto to reject the scan.

---

## 7. Troubleshooting

### Topics not appearing (`ros2 topic list` is empty or missing `/scan`, `/odom`, …)
- Confirm Isaac Sim is **Playing** (▶). If `/clock` is not ticking, nothing else will flow.
- Confirm **`ROS_DOMAIN_ID=1`** on **both** sides (`echo $ROS_DOMAIN_ID` in WSL2). A mismatch silently hides all topics.
- Confirm the **Fast DDS Discovery Server** is running in WSL2 and that Isaac is pointed at it as a CLIENT. `launch_all.cmd` starts the server and patches Isaac's client address each launch — if you started Isaac another way, the address may be stale.
- Confirm `rmw_fastrtps_cpp` is the active RMW (`echo $RMW_IMPLEMENTATION`). Re-source `config/source_ros_wsl.sh`.

### EKF / SLAM stops after pressing Stop or Play in Isaac
- Pressing **Stop/Play** in Isaac **resets `/clock`**, which kills the EKF (it depends on `use_sim_time` and a monotonic clock). After any Stop/Play, **restart the ROS launch**: `Ctrl+C` the `slam.launch.py` terminal and relaunch it.

### Isaac scene edits disappeared after a crash
- Isaac edits are **lost if you don't `Ctrl+S` before a crash**. Save the scene frequently while editing `scene_v4.usda`.

### slam_toolbox rejects the scan / map never builds
- Make sure the mapping pipeline is consuming **`/scan_fixed`** (produced by `scan_angle_fixer`), **not** the raw `/scan`. Verify the fixer is alive:
  ```bash
  ros2 topic hz /scan_fixed
  ```

### Robot does not move with teleop
- Check `/cmd_vel` is actually being published (`ros2 topic echo /cmd_vel`) and that the teleop terminal has keyboard focus.
- Confirm Isaac is Playing and subscribed to `/cmd_vel`.

---

## See also

- **[README.md](README.md)** — project overview, architecture, and status.
- **[docs/MASTER_PLAN.md](docs/MASTER_PLAN.md)** — full development plan and roadmap.
- Per-phase guides (Spanish): `docs/FASE1_GUIA_URDF.md`, `docs/FASE2_GUIA_ISAAC_SIM.md`, `docs/FASE3_GUIA_SLAM.md`.
