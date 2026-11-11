# Roomba — Setup, Operation and Calibration Guide

Step-by-step guide to download, build, run (simulation and real robot) and calibrate the Roomba. Overview and architecture are in [`README.md`](README.md).

**How to read the status marks in this guide**
- Commands come from the repository's own files and documentation (the four `roomba_*.launch.xml` files, `program_bringup/launch/README.md`, the manager node docstring, the calibration scripts, the firmware header).
- Nothing here has been executed by the author of this document. Treat every command as *unverified* until it works on your machine, and read the output of each step before moving on.
- `apt` package names are the usual ROS 2 Jazzy names; if one is not found, run `apt search <name>`.

**Contents**
1. [What you need](#1-what-you-need)
2. [Install ROS 2 and dependencies](#2-install-ros-2-and-dependencies)
3. [Download and build](#3-download-and-build)
4. [Run the simulation](#4-run-the-simulation)
5. [Hardware setup](#5-hardware-setup) (wiring, ESP32 firmware, Pi, udev, sensors)
6. [First hardware bring-up](#6-first-hardware-bring-up)
7. [Calibration](#7-calibration)
8. [Autonomous operation](#8-autonomous-operation)
9. [Viewing from a laptop](#9-viewing-from-a-laptop)
10. [Tests](#10-tests)
11. [Troubleshooting](#11-troubleshooting)
12. [Safety checklist](#12-safety-checklist)

---

## 1. What you need

| | Simulation only | Real robot |
|---|---|---|
| Computer | Ubuntu 24.04 PC | Raspberry Pi 5 (Ubuntu 24.04) + laptop for RViz |
| ROS | ROS 2 Jazzy | ROS 2 Jazzy |
| Extras | Gazebo (via `ros_gz`) | ESP32 + 2× BTS7960 + 2 motors with Hall encoders, YDLidar, BNO055, USB cables, Arduino IDE, a way to lift the robot's wheels off the ground |

ROS 2 Jazzy targets Ubuntu 24.04; use that OS on both the PC and the Pi.

---

## 2. Install ROS 2 and dependencies

### 2.1 ROS 2 Jazzy
Follow the official guide (desktop install): <https://docs.ros.org/en/jazzy/Installation.html>. Then:

```bash
sudo apt update
sudo apt install ros-dev-tools python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init        # skip if already done
rosdep update
```

### 2.2 ROS packages used by the launch files
```bash
sudo apt install \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server \
  ros-jazzy-twist-mux ros-jazzy-joy ros-jazzy-joy-teleop \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-tf-transformations ros-jazzy-backward-ros \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
```
`ros-jazzy-ros-gz` and `ros-jazzy-gz-ros2-control` are needed only for simulation (they are harmless on the robot, but you can skip them on the Pi).

### 2.3 Python libraries
Several nodes import libraries that are **not yet declared** in `package.xml`, so `rosdep` will not install them. Install them explicitly:

```bash
sudo apt install python3-numpy python3-scipy python3-opencv python3-skimage \
                 python3-serial python3-yaml python3-transforms3d
pip install --break-system-packages smbus2        # BNO055 over I2C (robot only)
```
(`python3-transforms3d` is required by `tf_transformations`.)

---

## 3. Download and build

> **Clone into `~/roomba`.** `manager_node`, `map_validator` and `data_logger` use hard-coded paths `~/roomba/saved_maps` and `~/roomba/collected_data`. Cloning anywhere else makes map saving, resuming and logging look in the wrong place.

```bash
git clone https://github.com/NOAA-dev/roomba ~/roomba
cd ~/roomba
```

### 3.1 YDLidar SDK (robot only, one-time)
`ydlidar_ros2_driver` needs the YDLidar-SDK library installed on the machine that builds it (the Pi). Follow the vendor steps in `src/ydlidar_ros2_driver/README.md` ("Build & Install YDLidar SDK"): clone <https://github.com/YDLIDAR/YDLidar-SDK>, then in its `build` directory run `cmake ..`, `make`, `sudo make install`.
On a simulation-only PC you can skip this and build without that package (see below).

### 3.2 Build
```bash
cd ~/roomba
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y     # optional; installs what package.xml declares
colcon build --symlink-install
source install/setup.bash
```
- `--symlink-install` makes edits to YAML and Python files take effect without rebuilding.
- Simulation-only PC without the lidar SDK: add `--packages-skip ydlidar_ros2_driver`. The simulation does not use that driver, but `program_bringup` lists it as a run dependency, so `rosdep` may complain; ignore with `-r`.
- Pi 5 memory: if the build is killed, limit parallelism, e.g. `colcon build --symlink-install --parallel-workers 2`.
- Add `source ~/roomba/install/setup.bash` to `~/.bashrc` once the build works.

---

## 4. Run the simulation

```bash
ros2 launch program_bringup roomba_sim.launch.xml
```
This starts Gazebo (server only, `small_house.world`), spawns the robot, and runs the whole stack: simulated clock, bridge, joystick chain, SLAM, EKF, A\*, pure pursuit, validator, explorers, manager, data logger and RViz.

There are four self-contained launch files (every node is written in the file itself):

| File | What runs |
|---|---|
| `roomba_sim.launch.xml` | Gazebo simulation, full stack |
| `roomba_prod.launch.xml` | real robot (run on the Pi), full stack, RViz off |
| `roomba_odom_test.launch.xml` | hardware odometry test: ESP32 + BNO055 + controllers + `cmd_relay` only |
| `roomba_nav_test.launch.xml` | hardware navigation test: lidar, SLAM, EKF, A\*, pure pursuit, validator, `twist_mux`, `manual_test_bringup`, data logger |

Settings are launch arguments with defaults at the top of each file. List them and override on the command line:
```bash
ros2 launch program_bringup roomba_sim.launch.xml --show-args
ros2 launch program_bringup roomba_sim.launch.xml world:=/path/to/other.world rviz:=false
```
- `sim` and `prod` have an `enable_<node>` switch for joystick, slam, ekf, global_planner, local_planner, map_validator, frontier_explorer, reactive_explorer, manager and data_logger (all default `true`).
- `sim` also has `world`, `gz_flags` (`-r -s -v 1`), `rviz`, `rviz_config`, `slam_config`, `ekf_config`.
- `prod` also has `enable_lidar`, `enable_imu`, `serial_port`, `baud_rate`, `rviz` (default `false`) plus the config overrides.
- `nav_test` has `serial_port`, `baud_rate`, `rviz`, `enable_imu`, `enable_data_logger`; `odom_test` has `serial_port`, `baud_rate`, `rviz`, `enable_imu`.

An unknown argument stops the launch with an error. After editing a launch file or config, run `colcon build --packages-select program_bringup` (not needed with `--symlink-install`).

---

## 5. Hardware setup

### 5.1 Wiring (from the firmware header)
| Function | ESP32 GPIO |
|---|---|
| Motor A (**right** wheel) — BTS7960 RPWM / LPWM | 32 / 33 |
| Motor A encoder C1 / C2 | 16 / 17 |
| Motor B (**left** wheel) — BTS7960 RPWM / LPWM | 25 / 26 |
| Motor B encoder C1 / C2 | 18 / 19 |

- The BTS7960 R_EN/L_EN pins are assumed tied high (always enabled) on the author's board. Check yours.
- PWM is 20 kHz, 8-bit; firmware limits output to `PWM_MAX 200`.
- If a wheel runs backwards on first test, see [Troubleshooting](#11-troubleshooting).

### 5.2 Flash the ESP32 firmware
1. Install the Arduino IDE and the **ESP32 board package, core v2.x** (the firmware uses the classic `ledcSetup/ledcAttachPin` API; on core v3.x those calls must be changed — see the comment in the firmware).
2. Open `esp32_codes/dual_motor_controller_FINAL.txt`, paste it into a new sketch (the file has a `.txt` extension; save it as `.ino`), select your ESP32 board and port, and upload.
3. Open the serial monitor at **115200 baud, newline**. Type `?` to list commands.
4. **First run after flashing (wheels lifted):**
   ```
   V1 30      → VEL A must come back positive and settle near 30 deg/s
   V2 30      → VEL B must come back positive and settle near 30 deg/s
   S          → ramped stop
   ```
   If a velocity is negative, stop and fix wiring/encoder polarity before continuing.
5. Commands you will use (full list in the firmware header): `V1/V2/VV <deg/s>` velocity, `S` stop, `Z` zero encoders, `T <ms>` telemetry period (use `T 40` if lines look garbled), `P` print status.

The firmware stops the motors if no velocity command arrives for 500 ms, blocks drive opposite to measured motion above 30 deg/s, and latches a FAULT (clear with `S`) if a wheel keeps moving against its target.

### 5.3 Raspberry Pi setup
```bash
sudo usermod -aG dialout,i2c $USER        # serial + I2C access; log out and in afterwards
sudo raspi-config                         # Interface Options → enable I2C
ls /dev/i2c-1                             # must exist
sudo apt install i2c-tools && i2cdetect -y 1   # BNO055 shows at 0x28 or 0x29
```
(If `raspi-config` is not available on your Ubuntu image, enable I2C by adding `dtparam=i2c_arm=on` to the Pi's boot `config.txt`.)

### 5.4 Stable device names (udev)
The ESP32 and the YDLidar both use CP2102 USB-serial chips with identical IDs (`10c4:ea60`, serial `0001`), so the repository's rules tell them apart **by physical USB port**:

| Device | Symlink | Port in the rule file |
|---|---|---|
| ESP32 | `/dev/roomba_esp32` | `KERNELS=="4-1"` |
| YDLidar | `/dev/roomba_lidar` | `KERNELS=="2-1"` |

```bash
sudo cp ~/roomba/src/program_bringup/udev/99-roomba-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/roomba_esp32 /dev/roomba_lidar
```
The port paths are specific to the author's Pi. If the links do not appear, find your own paths:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS      # repeat for each device, one plugged at a time
```
then edit the two `KERNELS==` values in the copied rule. Label the cables at the Pi end: moving a cable to another port silently swaps or removes the names.

### 5.5 Sensors
- **YDLidar:** configured in `src/ydlidar_ros2_driver/params/ydlidar.yaml` (`port: /dev/roomba_lidar`, 115200 baud, single-channel, 10 Hz, range 0.1–12 m, `inverted: true`). If your lidar model differs, use the matching file in `params/` and update the launch.
- **BNO055:** I²C bus 1, address from the node default; runs in IMUPLUS mode (no absolute heading). **Keep the robot still for the first ~2 s after launch**: the node waits 1.5 s, then takes the current heading as yaw = 0.
- **Camera:** no hardware camera support in this repository.

### 5.6 Configure for hardware
No mode switch is needed: use the `roomba_prod`, `roomba_nav_test` or `roomba_odom_test` launch file. Defaults are `serial_port:=/dev/roomba_esp32`, `baud_rate:=115200`, RViz off. Override on the command line if needed:
```bash
ros2 launch program_bringup roomba_prod.launch.xml serial_port:=/dev/ttyUSB0 enable_lidar:=false
```
The ESP32 hardware interface requests a 20 ms telemetry period (`T 20`) after connecting. `/joint_states` is published at the controller update rate (100 Hz, `controllers.yaml`), so its message spacing is 10 ms even though the ESP32 sends new wheel data about every 20 ms.

---

## 6. First hardware bring-up

Do these in order, with the **wheels lifted** until step 6.4.

**6.1 Link check** — launch the odometry-only test mode (no lidar, no SLAM):
```bash
ros2 launch program_bringup roomba_odom_test.launch.xml
```
This starts only the robot description, `ros2_control` + ESP32 interface, controllers, BNO055 IMU and `cmd_relay`. No lidar, EKF, SLAM, planners or data logger.

**6.2 Check topics** (second terminal, workspace sourced):
```bash
ros2 control list_hardware_interfaces      # left/right wheel velocity cmd + position/velocity state
ros2 topic hz /joint_states                # expect ~100 Hz (controller update rate)
ros2 topic echo /odometry --once
ros2 topic echo /imu --once
```
Spin a wheel by hand: `/joint_states` positions must change, left and right with the **same sign for forward motion**.

**6.3 Send a tiny command (wheels lifted):**
```bash
ros2 run helpers manual_motion_test        # in a new terminal
> velocity 0.05 0 3                        # 0.05 m/s, 0 rad/s, 3 s
```
Both wheels must turn **forward**. If the robot would move backwards or spin the wrong way, see Troubleshooting.

**6.4 Floor test:** put the robot on the floor in a clear area and use `straight 50`, `turn 90`, `stop` (see [Calibration](#7-calibration)).

Stopping: type `stop` in the tool, press Ctrl-C in the launch terminal, or send `S` from the ESP32 serial monitor. The firmware also stops 500 ms after the last command, and `cmd_relay` zeroes the wheels 0.5 s after the last velocity message.

---

## 7. Calibration

Order matters: motors → geometry → odometry → IMU → SLAM sanity. Each stage should pass before the next.

### 7.1 Motor calibration (ESP32, open loop, wheels lifted)
Tools in `esp32_codes/calibration/` produce the speed-to-PWM tables, deadband, PID starting values and brake timings used in the firmware.

```bash
pip install pyserial numpy matplotlib
cd ~/roomba/esp32_codes/calibration
python3 calibrate_host.py --sim --yes --tests all --reps 1        # dry run, no hardware
```
1. Flash `calibration_suite.ino` (same Arduino workflow as 5.2; it speaks **460800 baud**, so close other serial monitors).
2. Lift the wheels, then:
   ```bash
   python3 calibrate_host.py --port /dev/ttyUSB0 --tests all --out results
   python3 calibrate_host.py --port /dev/ttyUSB0 --tests t2,t3 --motors A --out results   # subset
   ```
   Use `--out <existing dir>` to continue a session (`--resume` for T3).
3. Tests (all open loop): `t0` encoder counts per rev (hand rotation), `t1` direction, `t2` deadband, `t3` PWM→speed sweep, `t4` step response, `t5` velocity-estimator noise, `t6` braking/reversal interlock, `t7` left/right matching.
4. Results accumulate in `results/results.json` with raw CSVs. Copy the fitted tables/constants into the firmware (`COUNTS_PER_REV_*`, feedforward tables, PID, ramps), then re-flash `dual_motor_controller_FINAL.txt`. **This copy step is manual.**
5. Closed-loop validation (T8) and loaded-floor repeat (T9) need the real control firmware, not the calibration rig.

Firmware logic can be exercised without hardware using the host simulator:
```bash
cd ~/roomba/esp32_codes/calibration/sim
g++ -std=c++17 -O1 -Iinc -x c++ sim.cpp -o sim && ./sim 1 1 step     # also: rev | fault | low | oldgains
```

> **Check before trusting odometry:** the firmware currently sets `COUNTS_PER_REV_A = 5075.0` and `COUNTS_PER_REV_B = 5077.0`, while its comments, the calibration suite and `calibrate_host.py` use 5070. Decide which is correct (T0 hand-rotation count) and make the code and comments agree. A wrong value scales wheel distance by 0.1–0.14 %.

### 7.2 Robot geometry
Measure with calipers/ruler and enter in `src/program_bringup/config/robot_dimensions.yaml` (entries marked `[MEASURED]` are measured, `[LEGACY]` are not yet re-measured):

| Key | Meaning |
|---|---|
| `robot.drive_wheel.radius_m` / `diameter_m` | wheel size (diameter must equal 2 × radius) |
| `robot.drive_wheel.track_width_m` | wheel-centre to wheel-centre (not the body width) |
| `robot.footprint.length_m / width_m` | body outline |
| `robot.lidar.x_m`, `robot.imu.x_m` | sensor positions; `lidar.x_m` applies to the hardware model only |
| `limits.max_wheel_speed_m_s` | per-wheel speed cap, sim and hardware (0.25) |

`controllers.yaml` also contains `wheel_radius` and `wheel_separation` (for the unused `diff_drive_controller`); keep them equal to the shared file. `src/helpers/test/test_robot_dimensions.py` checks the consistency. If you change the speed cap, also check `joystick_teleop.yaml`, `twist_mux_joy.yaml` and `manual_motion_test` limits, which hold derived numbers.

Rebuild `urdfs`/`program_bringup` if you are not using `--symlink-install`.

### 7.3 Odometry calibration (floor)
Use the odometry-only mode from 6.1 on a flat, high-friction surface with a tape measure and a marked start line.

```bash
ros2 launch program_bringup roomba_odom_test.launch.xml
ros2 run helpers manual_motion_test
```
Terminal commands: `straight <cm>`, `turn <degrees>`, `velocity <m/s> <rad/s> <s>`, `stop`, `odom`, `help`, `quit`. After each motion the tool prints requested vs measured distance/angle and a settled-odometry report, with an IMU/EKF cross-check.

**Straight-line scale** — drive `straight 200`, measure the real distance, repeat ≥ 5 times and average:
```
radius_new = radius_old × (measured_real_distance / odometry_reported_distance)
```
Update `radius_m` and `diameter_m` (= 2 × radius) in `robot_dimensions.yaml` and `wheel_radius` in `controllers.yaml`.

**Rotation scale** — mark the heading, run `turn 360` (or `turn 90` ×4), compare with the real angle, repeat ≥ 5 times:
```
track_new = track_old × (odometry_reported_angle / real_angle)
```
Update `track_width_m` in the shared file and `wheel_separation` in `controllers.yaml`.

Repeat the two checks after each change. Target values used in this project's roadmap: straight-line error under 2 %, spin error under 3° per revolution, with repeatable results. If runs differ from each other by more than the correction you are applying, the cause is not scale (slip, ramps, surface, battery sag) — investigate before changing parameters.

**Note** — the reference floor test recorded in the project notes: one commanded wheel revolution (r = 3.5 cm, theoretical 22.0 cm) travelled 22.3 cm.

### 7.4 IMU check
```bash
ros2 topic echo /imu --field orientation          # robot still: yaw ≈ 0 shortly after start
```
Rotate the robot **counter-clockwise** by hand: yaw must **increase** (REP-103). The mount correction lives only in `src/helpers/helpers/bno055.py` (`MOUNT_MATRIX`) and is covered by `test_bno055_mount.py`; change it only if the sign or axes are wrong on your mounting. IMUPLUS mode has no magnetometer, so yaw drifts slowly; the EKF fuses only yaw from the IMU.

### 7.5 EKF and SLAM sanity
Run the autonomy test mode (section 8.1) and check, in RViz with the Fixed Frame set to `odom`:
- driving a 1 m square returns near the start in `odom`;
- `map → odom` changes smoothly (jumps appear only at loop closures);
- `/scan` aligns with the map walls.

`ekf.yaml` and the covariances in `cmd_relay` are initial estimates; tune them from logged runs (`data_logger` output plus `src/helpers/helpers/visualize_data_logs.py`) after the scale calibration above, not before.

---

## 8. Autonomous operation

### 8.1 Supervised navigation test (RViz goals, no exploration)
```bash
ros2 launch program_bringup roomba_nav_test.launch.xml
```
Starts lidar, SLAM, EKF, A\*, pure pursuit, `map_validator`, `twist_mux`, `manual_test_bringup` (which activates slam_toolbox/map saver and enables the planners) and the **data logger** (`enable_data_logger:=false` to turn it off). There is no manager, no explorers and no joystick. Give goals from RViz (laptop, section 9) with **2D Goal Pose**, or from a terminal:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```
Do **not** run `manual_motion_test` at the same time: it also broadcasts `odom → roomba` and clashes with the EKF. Keep a hand on the power switch for the first runs.

### 8.2 Full stack
```bash
ros2 launch program_bringup roomba_prod.launch.xml
```
The manager runs a state machine (also published on `/manager/state`):
```
STARTUP -> no saved map: FULL_EXPLORATION
        -> saved map:    AWAIT_INIT_POSE -> VERIFY_SAVED_MAP -> SWEEP
FULL_EXPLORATION / SWEEP -> RETURN_HOME -> AWAIT_CONFIRM -> SAVING -> COMPLETE (or PENDING_PROMOTION)
FULL_EXPLORATION -> RETURN_HOME -> DWELL_HOME -> FULL_EXPLORATION     (periodic revisit, revisit_distance_m > 0)
```
Commands (plain ROS 2, no joystick needed):
```bash
ros2 topic echo /manager/state
# resuming a saved map: tell the manager where the robot is in the saved map frame
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 2.0}, orientation: {z: 0.0, w: 1.0}}}}"
ros2 service call /manager/return_home       std_srvs/srv/Trigger {}   # from FULL_EXPLORATION, SWEEP or AWAIT_CONFIRM
ros2 service call /manager/confirm_save      std_srvs/srv/Trigger {}   # only in AWAIT_CONFIRM, and only if the map is VALID
ros2 service call /manager/promote_candidate std_srvs/srv/Trigger {}   # only in PENDING_PROMOTION
```
There is no service to decline a save: simply do not call `confirm_save`.

**Map storage** (`~/roomba/saved_maps`): the validator writes each saved map as a *candidate* in `saved_maps/candidates/map_<stamp>.{yaml,pgm,data,posegraph}` and refuses to save unless the map is valid and `map → odom` is not jumping. `saved_maps/CURRENT` is a one-line pointer to the map that will be resumed on the next start; `promote_candidate` moves it. Unpromoted candidates are never resumed. A map needs all four files (`.yaml`, `.posegraph`, `.data`, `.pgm`).

Logs go to `~/roomba/collected_data` (section 8.5).

### 8.3 Joystick
With `enable_joystick:=true` (default in `prod` and `sim`), a gamepad (`joy` → `joy_teleop` → `twist_mux`) has priority 99 over autonomy (90). Hold the deadman button (index 5) to drive; full stick = 0.25 m/s and 2.08 rad/s. Releasing the deadman stops the robot.

### 8.4 Stopping
Joystick deadman release, Ctrl-C on the launch, or power cut. There is no hardware e-stop logic in this repository; add a physical power switch.

### 8.5 Logging and analysing experiments
`data_logger` (enabled in `prod`, `sim` and `nav_test`) writes CSVs to `~/roomba/collected_data`: odometry, `map → roomba` TF, joint states, wheel commands, validated-map summary (valid, fail reason, map→odom jump, loop-closure flag) and map grids. `src/helpers/helpers/visualize_data_logs.py` plots one run.

To compare many runs, copy each run's data and saved map into `experiment/run_N/`, add a filled `measured.txt` (template: `tools/measured_template.txt`), then:
```bash
python3 tools/experiment_analyser.py experiment/            # writes experiment/results/
python3 tools/experiment_analyser.py --make-template        # print a blank measured.txt
python3 tools/experiment_analyser.py --selftest             # synthetic check, no data needed
```
Output: `results.csv` (one row per run), `summary.txt`, `report.html`, plots. It reports goal/localization error, map→odom drift and loop-closure steps, wheel and body velocity tracking gains, stationary yaw drift, stops/in-place turns and map size. Optional arguments: `--wheel-radius`, `--track-width`, `--loop-jump-m`, `--loop-jump-rad`, `--no-plots`, `--legacy-plots`. Needs `numpy` and `matplotlib`.

---

## 9. Viewing from a laptop

Run RViz on a laptop so the Pi's CPU is spent on the robot.

1. Pi: launch `roomba_prod.launch.xml` (RViz stays off, `rviz:=false`).
2. Both machines: ROS 2 Jazzy, same network/subnet, same `ROS_DOMAIN_ID` (put the export in `~/.bashrc` on both).
3. Do not set `ROS_LOCALHOST_ONLY=1` or `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; use the same `RMW_IMPLEMENTATION`; allow UDP between the machines; avoid Wi-Fi client isolation.
4. Laptop:
   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=<same as the Pi>
   ros2 topic list                        # /tf, /scan, /map, /robot_description should appear
   rviz2 -d ~/roomba/src/urdfs/rviz/default.rviz        # manual_test.rviz for the odometry test
   ```
   Do not set `use_sim_time` on the laptop. If the Fixed Frame `map` has no data yet, switch it to `odom`.
5. Nothing visible? Multicast is probably blocked. Use a wired link/hotspot, or a Fast DDS discovery server (`ROS_DISCOVERY_SERVER=<pi-ip>:11811`).

---

## 10. Tests

Unit and launch tests are in `src/helpers/test/` (`test_robot_dimensions.py`, `test_relay_safety.py`, `test_bno055_mount.py`) plus ament lint stubs.
```bash
cd ~/roomba
colcon test --packages-select helpers
colcon test-result --verbose
```
These cover configuration consistency and relay/IMU logic only. They do not test the robot, SLAM or the planners.

---

## 11. Troubleshooting

| Symptom | Likely cause / action |
|---|---|
| Launch stops immediately with an argument error | unknown or misspelled launch argument — check `--show-args` |
| `/dev/roomba_esp32` missing | udev rule not installed, or USB port differs — redo 5.4 |
| `ESP32 serial link ... not connected` repeats | wrong port, no permission (`dialout` group), ESP32 not flashed, or Serial Monitor holding the port |
| Joint states never change | no telemetry parsed yet; send `P` in a serial monitor; check baud 115200 and the `T` setting (`T 40` if garbled) |
| Robot drives backwards / spins the wrong way | swap `left_motor_channel`/`right_motor_channel` in `src/urdfs/urdf/ros2_hardwareinterface.xacro` (currently left = B, right = A), or fix wiring/encoder polarity, then re-run 6.2–6.3 |
| Wheels stop after ~0.5 s | expected: command timeout. Commands must be refreshed continuously |
| Wheel stops and a FAULT is reported | polarity/runaway guard latched; fix the cause, send `S` |
| `Wheel command limited` warnings | requested speed exceeds 0.25 m/s per wheel; the relay scales both wheels |
| No `/imu` or I²C errors | I²C not enabled, wrong bus/address, `smbus2` missing, user not in `i2c` group |
| Yaw jumps at start | robot moved during the 1.5 s settle |
| `ImportError` for `cv2`, `skimage`, `scipy`, `smbus2` | section 2.3 libraries missing |
| No lidar scan | `/dev/roomba_lidar` missing, YDLidar-SDK not installed when building, wrong params file |
| TF errors `map`→`roomba` | SLAM not active yet (manager still starting) or EKF not running |
| Saved map not resumed | `~/roomba/saved_maps` missing, or the four-file map set is incomplete |
| RViz empty on the laptop | domain ID, firewall or multicast problem (section 9) |

---

## 12. Safety checklist

- First run every new firmware or wiring change with the **wheels lifted**.
- Keep the robot in a clear area for floor tests; stay within reach of the power switch.
- Point-to-point navigation has been tested on the robot (15 floor runs: mean goal error about 6 cm, max 11 cm), but turning is slower than commanded (angular velocity gain about 0.64) and yaw drifts about 4.6 °/min while stopped: keep supervising every autonomous run.
- Keep the 0.25 m/s cap until the odometry and obstacle behaviour have been verified at that speed; the robot's unloaded top speed on the author's firmware is about 0.30 m/s, so there is little headroom under load or battery sag.
- Do not move the lidar/ESP32 USB cables between ports without updating the udev rules.
