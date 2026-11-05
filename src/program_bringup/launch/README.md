# program_bringup

Launch files and configuration for the roomba, in simulation and on the real robot.

## Start

Four launch files, each self-contained (every node is written in the file itself):

```bash
ros2 launch program_bringup roomba_sim.launch.xml        # Gazebo simulation
ros2 launch program_bringup roomba_prod.launch.xml       # real robot (run on the Pi)
ros2 launch program_bringup roomba_odom_test.launch.xml  # hardware odometry test
ros2 launch program_bringup roomba_nav_test.launch.xml   # hardware navigation test
```

| file | what runs |
|---|---|
| `roomba_sim` | Gazebo + `gz_ros2_control`, bridge, simulated clock, joystick chain, SLAM, EKF, A*, pure pursuit, validator, explorers, manager, RViz, data logger |
| `roomba_prod` | `ros2_control_node` + `roomba_hardware` (ESP32), YDLidar, BNO055, system clock, joystick chain, SLAM, EKF, A*, pure pursuit, validator, explorers, manager, data logger (RViz off) |
| `roomba_odom_test` | ESP32 + BNO055 + controllers + `cmd_relay`. No lidar, EKF, SLAM, planners or logger. Drive with `ros2 run helpers manual_motion_test` in a second terminal |
| `roomba_nav_test` | Hardware + lidar, SLAM, EKF, A*, pure pursuit, map_validator, twist_mux, `manual_test_bringup`. Goals from RViz (2D Goal Pose). No manager, explorers, joystick or logger |

Settings are launch arguments with defaults at the top of each file. List them with
`ros2 launch program_bringup roomba_prod.launch.xml --show-args`, override on the command line:

```bash
ros2 launch program_bringup roomba_prod.launch.xml enable_lidar:=false enable_manager:=false
ros2 launch program_bringup roomba_sim.launch.xml world:=/path/to/other.world rviz:=false
```

`sim` and `prod` have an `enable_*` switch per node (joystick, slam, ekf, global_planner,
local_planner, map_validator, frontier_explorer, reactive_explorer, manager, data_logger).
`prod` also has `enable_lidar`, `enable_imu`, `serial_port`, `baud_rate`.

Robot geometry is separate: `config/robot_dimensions.yaml` (read by the xacro,
`cmd_relay`, the A* planner, pure pursuit and the reactive explorer).

After editing, run `colcon build --packages-select program_bringup` (or build once
with `--symlink-install` and edits apply immediately).

## Simulation vs hardware

| | `roomba_sim` | `roomba_prod` |
|---|---|---|
| Clock | simulated (`use_sim_time: true`) | system clock |
| Robot / ros2_control | Gazebo + `gz_ros2_control` | `ros2_control_node` + `roomba_hardware` (ESP32 over serial) |
| Sensors | Gazebo via the bridge | YDLidar + BNO055 |
| RViz | on this machine (`rviz:=true`) | off on the Pi, run it on the laptop |
| Wheel-speed cap | 0.25 m/s per wheel (`limits.max_wheel_speed_m_s` in `robot_dimensions.yaml`, both modes) |

## View the real robot from a laptop

The robot stack stays on the Pi; the laptop only runs RViz.

1. On the Pi launch `roomba_prod.launch.xml` (RViz stays off).
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
  roomba_sim.launch.xml
  roomba_prod.launch.xml
  roomba_odom_test.launch.xml
  roomba_nav_test.launch.xml
config/
  robot_dimensions.yaml        robot geometry (single source of truth)
  controllers.yaml, ekf.yaml, slam_toolbox.yaml, ...
```
