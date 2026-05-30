---
title: cargo_bot
---

# 🤖 cargo_bot

**Autonomous indoor cargo robot** — built sim-first in NVIDIA Isaac Sim 5.1 + ROS 2 Humble (WSL2), bridged over Fast DDS.

> 🎯 North-star: carry laundry to the laundry room using the building elevator (multi-floor navigation).

<!-- TODO: replace this line with the demo GIF/video — it is the hero of the page -->
<!-- ![cargo_bot demo](assets/demo.gif) -->

## What it does today

- 🗺️ **2D SLAM** with `slam_toolbox` → saved map of the room
- 🧭 **EKF sensor fusion** (wheel odometry + IMU yaw) via `robot_localization`
- 🏗️ Full **Isaac Sim ↔ ROS 2** bridge over Fast DDS Discovery Server
- 🚧 **Nav2** autonomous navigation — *in progress*

## Links

- 📦 [Source code & full README](https://github.com/AgustinPrietoValdez/cargo_bot_ws)
- 🗂️ [Roadmap board](https://github.com/users/AgustinPrietoValdez/projects/1)
- 🏷️ [Releases](https://github.com/AgustinPrietoValdez/cargo_bot_ws/releases)

## Tech

`ROS 2 Humble` · `NVIDIA Isaac Sim` · `C++` · `Python` · `Nav2` · `slam_toolbox` · `robot_localization` · `STM32` / `Raspberry Pi` (target hardware)

---

Built by **[Agustín Prieto Valdez](https://github.com/AgustinPrietoValdez)** — engineering student @ Aalborg University, focused on mobile robotics.
