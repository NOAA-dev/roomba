# roomba_hardware

`ros2_control` `SystemInterface` plugin that drives the real `roomba` base
over a serial link to an ESP32, as a drop-in real-hardware counterpart of
`gz_ros2_control/GazeboSimSystem`. Same two joints
(`base_left_wheel_joint`, `base_right_wheel_joint`), same `velocity`
command interface and `position`/`velocity` state interfaces, so the
`simple_velocity_controller` + `relay.py` + `ekf_node` + planner stack that
already works in sim runs unchanged against real hardware.

Selected via `use_sim:=false` on `roomba.urdf.xacro` (default `use_sim:=true`
keeps existing sim behavior); see `roomba_hardware.launch.xml` in
`program_bringup` for the real-hardware bringup.

## What it does, and doesn't, do

It only makes `/joint_states` contain real numbers from the ESP32 instead of
simulated ones. It does **not** do a second odometry integration --
`relay.py` + `ekf_node` already do that off of `/joint_states`, and that
chain is left as-is (this package/repo currently keeps
`simple_velocity_controller`, not `diff_drive_controller`, per the sim
setup).

## `<hardware><param>` entries (set in `ros2_hardwareinterface.xacro`)

| param                | default        | meaning                                                              |
|-----------------------|----------------|-----------------------------------------------------------------------|
| `serial_port`          | *(required)*   | e.g. `/dev/ttyUSB0`                                                   |
| `baud_rate`             | `115200`       | must match the ESP32 firmware                                         |
| `left_motor_channel`    | `A`            | which ESP32 motor (`A`/`B`) is `base_left_wheel_joint`                |
| `right_motor_channel`   | `B`            | which ESP32 motor (`A`/`B`) is `base_right_wheel_joint`                |

**The Motor A/B <-> left/right mapping is an unverified guess from the
firmware side.** If the robot spins in place or drives backwards on first
test, flip `left_motor_channel`/`right_motor_channel` in
`ros2_hardwareinterface.xacro` -- it's a one-line xacro change, not a
rebuild.

## Design notes

Protocol details below are verified against the actual firmware
(`dual_motor_controller_ros2_compatible`), not just the original spec doc --
two behaviors only became visible once the firmware source was available:

- **Command watchdog**: the firmware force-stops a motor that's in velocity
  mode if it doesn't see a V1/V2/VV refresh within `VEL_CMD_TIMEOUT_MS`
  (500ms in this firmware). Our 100ms keepalive (below) already gives a 5x
  margin under that; if `VEL_CMD_TIMEOUT_MS` ever changes, keep the
  keepalive comfortably below half of it.
- **Telemetry goes silent at rest**: the firmware only streams `POS`/`VEL`
  telemetry unprompted while at least one motor is out of idle mode --
  once both wheels settle to a stop, nothing more arrives on its own. `P`
  always returns a status line regardless of idle/active state, so
  `SerialLink` polls with `P` on a 200ms fallback timer whenever real
  telemetry hasn't shown up recently. Without this, `read()` would freeze
  on whatever velocity happened to be measured the instant a motor went
  idle (not guaranteed to be exactly 0) and hold it forever.
- **Direction/sign**: no correction needed on the ROS side. The firmware
  already normalizes Motor A and Motor B to the same "+command -> +counts"
  convention internally (the two are wired oppositely at the hardware
  level, intentionally, for a mirror-mounted diff-drive base) -- from this
  package's point of view both channels behave identically and
  symmetrically.

Other design notes:

- **Threading**: all blocking serial I/O (open/read/write/reconnect) happens
  on a private background thread owned by `SerialLink`. The
  `read()`/`write()` calls `controller_manager` makes at 100 Hz just
  copy/latch atomics, so they never block on the device.
- **Command rate-limiting**: `write()` doesn't put a new `V1`/`V2` on the
  wire every 10 ms call. It sends when the commanded speed changes by more
  than 1.0 deg/s since the last value actually sent, or every 100 ms
  regardless (keepalive, see the watchdog note above), whichever comes
  first -- see the comment above `kCommandChangeThresholdDegS` in
  `serial_link.cpp` for the full reasoning.
- **Telemetry parsing**: `POS`/`VEL` lines are parsed defensively (regex,
  tolerant of whitespace) and resynchronize on the next newline if a line
  doesn't match; anything else on the wire (acks, help text) is ignored.
  `read()` holds the last known state between telemetry lines rather than
  reporting zero.
- **Reconnect**: if the ESP32 isn't present at startup, or the link drops
  mid-session, `SerialLink` keeps retrying the open once a second in the
  background. `read()`/`write()` keep returning `OK` with last-known state
  rather than tearing down `ros2_control_node`.
- **Lifecycle**: `on_configure` opens (or starts retrying to open) the
  serial link, so states are already observable in `INACTIVE`. `on_activate`
  sends `Z` once (zeroes the encoder reference). `on_deactivate` sends `S`
  (ramped stop) so the robot doesn't coast on its last velocity command.

## Serial permissions

The ESP32 will typically show up as `/dev/ttyUSB0` or `/dev/ttyACM0`, owned
by `root:dialout` with group read/write. Add your user to the `dialout`
group once:

```bash
sudo usermod -aG dialout $USER
# log out/in (or `newgrp dialout`) for the group change to take effect
```

If the robot has multiple USB serial devices (LIDAR/IMU too) and
`/dev/ttyUSB0` isn't stable across reboots/replugs, add a udev rule keyed
on the ESP32's USB vendor/product ID (`udevadm info -a -n /dev/ttyUSB0` to
find them) so it always shows up under a fixed name, and point
`serial_port` at that instead.

## pluginlib export

```xml
<class name="roomba_hardware/ESP32SystemInterface"
       type="roomba_hardware::ESP32SystemInterface"
       base_class_type="hardware_interface::SystemInterface">
  <description>
    Real-hardware ros2_control SystemInterface for the roomba diff-drive
    base, talking to an ESP32 over serial.
  </description>
</class>
```

(already in `roomba_hardware.xml`, exported via
`pluginlib_export_plugin_description_file(hardware_interface roomba_hardware.xml)`
in `CMakeLists.txt`.)
