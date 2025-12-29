# 🤖 dddmr_navigation
# 🤖 dddmr_navigation

## 🚀 Latest Update: MID360 LiDAR Support
**Great news!** We have adapted the DDDMR Navigation Stack to support the MID360 360° LiDAR sensor (MID 360). This includes configuration files, launch samples, and tested workflows for mapping, localization and planning on Ackermann-type robots using MID360 pointclouds. See the detailed Ackermann guide for step-by-step instructions and parameter examples: `ACKERMANN_FULL_DEBUG_GUIDE.md`.

👉 Try the ACKERMANN setup or check the MID360-specific configs in `src/dddmr_lego_loam` and `src/dddmr_mcl_3dl`.

---

## 🚀 Highlights & New Demos
We continue to support a wide range of mobile platforms (including Ackermann steering vehicles and quadrupeds) and sensors. Recent additions include:

- MID360 (360° LiDAR) mapping & localization configs
- Ackermann-specific planning and control examples
- Demo images and sample outputs (embedded below)

<p align='center'>
  <img src="image/map.png" alt="map demo" width="640" />
</p>

<p align='center'>
  <img src="image/ground.png" alt="ground segmentation demo" width="320" style="margin-right:12px;" />
  <img src="image/pcd and pose.png" alt="pcd and pose demo" width="320" />
</p>

---

> **Note:** DDDMR Navigation Stack is designed to extend what 2D stacks (e.g., Nav2) can do — multi-level mapping, ramp navigation, 3D perception-based planning and semantic clearings.

## ✅ Why choose DDDMR (3D Navigation)?

- Familiar workflow for 2D navigation users: map → localize → plan & navigate
- Handles complex terrains (ramps, multi-floor environments)
- Works on cost-effective hardware (e.g., MID360, 16-line lidar, NUC/Jetson)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation_diagram.png" width="780" height="560"/>
</p>

🤝 We thank contributors and projects like [Navigation](https://github.com/ros-planning/navigation) and [Navigation2](https://github.com/ros-navigation/navigation2).

If you're getting started, check the beginner's guide: [dddmr_beginner_guide](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide)

## 🏁 Detailed documentation for each package
<details><summary> <b>💡 Click me to see Mapping</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam
</details>
<details><summary> <b>💡 Click me to see Localization</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_mcl_3dl
</details>
<details><summary> <b>💡 Click me to see Perception</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_perception_3d
</details>
<details><summary> <b>💡 Click me to see Global planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_global_planner
</details>
<details><summary> <b>💡 Click me to see Local planner</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_local_planner
</details>
<details><summary> <b>💡 Click me to see Move base</b> </summary>
https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_p2p_move_base
</details>

## Demonstrations of DDD navigation functions
<table align='center'>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping.gif" width="400" height="260"/><p align='center'>3D mapping</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/global_planner.gif" width="400" height="260"/><p align='center'>3D global planning</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/local_planner.gif" width="400" height="260"/><p align='center'>3D local planning</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation.gif" width="400" height="260"/><p align='center'>3D navigation</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Unitree G4)</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="400" height="260"/><p align='center'>Support vairant sensors (Depth Camera)</p></td>
  </tr>
</table>

