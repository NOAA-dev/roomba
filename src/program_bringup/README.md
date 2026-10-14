# program_bringup

Launch files and configuration for the roomba, in simulation and on the real robot.

## Start

```bash
ros2 launch program_bringup startup.launch.py
```

`config/startup.yaml` is the one file that selects the mode; no launch
arguments or source edits are needed:

| setting | meaning |
|---|---|
| `mode: sim` / `hardware` | Gazebo simulation, or ESP32 + YDLidar + BNO055 on the robot |
| `sim:` | world, Gazebo flags, RViz |
| `hardware:` | serial port, baud rate, RViz, lidar on/off, IMU on/off |
| `slam_config`, `ekf_config`, `rviz_config` | override the default config files (`null` = default) |
| `nodes:` | start/skip each node (joystick, slam, ekf, planners, explorers, manager, data logger) |

Robot geometry is separate: `config/robot_dimensions.yaml` (read by the xacro,
`cmd_relay` and the A* planner).

A typo in the file (unknown key, wrong type, missing file) stops the launch
with a message naming the problem. To keep several setups, copy the file and
use `startup_config:=/path/to/other.yaml`. After editing, run
`colcon build --packages-select program_bringup` (or build once with
`--symlink-install` and edits apply immediately).

## Simulation vs hardware

| | `mode: sim` | `mode: hardware` |
|---|---|---|
| Clock | simulated (`use_sim_time: true`) | system clock |
| Robot / ros2_control | Gazebo + `gz_ros2_control` | `ros2_control_node` + `roomba_hardware` (ESP32 over serial) |
| Sensors | Gazebo via the bridge | YDLidar + BNO055 |
| RViz | on this machine (`sim.rviz`) | off on the Pi (`hardware.rviz: false`), run it on the laptop |
| Wheel-speed cap | 0.25 m/s per wheel (`limits.max_wheel_speed_m_s` in `robot_dimensions.yaml`, both modes) |

## View the real robot from a laptop

The robot stack stays on the Pi; the laptop only runs RViz.

1. On the Pi set `mode: hardware` (RViz stays off) and launch as above.
2. Both machines need ROS 2 Jazzy on the same network/subnet and **the same
   `ROS_DOMAIN_ID`** (unset = 0). Put the export in `~/.bashrc` on both, and
   in whatever starts the launch on the Pi.
3. Keep discovery open on both: do not set `ROS_LOCALHOST_ONLY=1` or
   `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (Jazzy default is `SUBNET`), and
   use the same `RMW_IMPLEMENTATION` on both (default `rmw_fastrtps_cpp`).
   Allow UDP between the machines (firewall) and avoid Wi-Fi client isolation.
4. On the laptop (no workspace build needed; the config only uses stock RViz
   displays), copy `src/urdfs/rviz/default.rviz` and run:

   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=<same as the Pi>
   ros2 topic list                       # should list /tf, /scan, /map, /robot_description
   rviz2 -d default.rviz
   ```

   The laptop uses the system clock too, so do not set `use_sim_time` there.
   If the Fixed Frame `map` has no data yet (SLAM not started), switch it to
   `odom`. The camera display stays empty on hardware.
5. If `ros2 topic list` on the laptop shows nothing, multicast discovery is
   probably blocked: use a wired link or a hotspot that allows multicast, or
   a Fast DDS discovery server (`ROS_DISCOVERY_SERVER=<pi-ip>:11811`).

## Layout

```
launch/
  startup.launch.py            entry point - reads config/startup.yaml
  modules/                     building blocks included by startup.launch.py
    robot              robot_state_publisher (+ ros2_control_node on hardware)
    simulation         Gazebo, robot spawn, ROS<->Gazebo bridge       (sim only)
    hardware_sensors   YDLidar + BNO055 IMU                           (hardware only)
    controllers        ros2_control controller spawners
    localization       robot_localization EKF
    slam               map_saver_server + slam_toolbox
    navigation         A* global planner + pure pursuit local planner
    exploration        map validator, frontier/reactive explorers, manager node
    tools              RViz + data logger
    joystick           joy -> joy_teleop -> twist_mux -> cmd_relay
config/
  startup.yaml                 mode and node selection (the file you edit)
  robot_dimensions.yaml        robot geometry (single source of truth)
  controllers.yaml, ekf.yaml, slam_toolbox.yaml, ...
```

Every module declares its arguments with a default and a description.
