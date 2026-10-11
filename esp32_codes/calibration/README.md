# Motor calibration + firmware sim

- `calibration_suite.ino` - open-loop rig firmware (flash, then run the host script)
- `calibrate_host.py` - Windows/Linux host: `py calibrate_host.py --port COMx --out results` (`--resume` for T3)
- `sim/` - host-side build of `../dual_motor_controller_FINAL.txt` against a plant model from the measured tables:
  `cd sim && g++ -std=c++17 -O1 -Iinc -x c++ sim.cpp -o sim && ./sim 1 1 step|rev|fault|low|oldgains`
  (Arduino stub in `sim/inc/Arduino.h`; no hardware needed)

Firmware v4.1 constants come from T1-T7 results: CPR 5070, per-motor FF tables, start kick 41 PWM,
Kp0.15/Ki0.5/Kd0, accel 600 / decel 1200 deg/s2, reversal interlock at 30 deg/s, PWM_MAX 200.
