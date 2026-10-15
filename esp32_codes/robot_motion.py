import math
import re
import time
import serial

PORT = "COM6"
BAUD = 115200

WHEEL_RADIUS_CM = 3.5
TRACK_WIDTH_CM = 24.0

SPEED_DEG_S = 45.0
ACCEL_DEG_S2 = 150.0
DECEL_DEG_S2 = 300.0

VELOCITY_TOLERANCE = 4.0
POSITION_STABILITY_DEG = 0.5
STABLE_TIME_S = 1.0
TELEMETRY_TIMEOUT_S = 0.5
MOTION_TIMEOUT_S = 90.0

STATUS_PATTERN = re.compile(
    r"POS A=([-+0-9.]+)deg\s+"
    r"VEL A=([-+0-9.]+)deg/s.*?"
    r"POS B=([-+0-9.]+)deg\s+"
    r"VEL B=([-+0-9.]+)deg/s"
)


def send_command(esp, command):
    print(f">> {command}")
    esp.write((command + "\n").encode("ascii"))
    esp.flush()


def parse_status(line):
    match = STATUS_PATTERN.search(line)
    if not match:
        return None

    a_pos, a_vel, b_pos, b_vel = map(float, match.groups())

    return {
        "right_pos": a_pos,
        "right_vel": a_vel,
        "left_pos": b_pos,
        "left_vel": b_vel,
    }


def read_initial_status(esp, timeout=2.0):
    """Request P and collect the latest valid status."""
    esp.reset_input_buffer()
    send_command(esp, "P")

    deadline = time.monotonic() + timeout
    latest = None

    while time.monotonic() < deadline:
        line = esp.readline().decode("utf-8", errors="ignore").strip()
        status = parse_status(line)

        if status is not None:
            latest = status

    if latest is None:
        raise TimeoutError("No valid status received from ESP32.")

    return latest


def wait_for_motion_to_settle(esp, start):
    """
    Monitor ESP32 telemetry.
    If telemetry goes quiet, request P.
    Finish when movement has occurred and the reported
    positions and velocities remain stable.
    """
    started_at = time.monotonic()
    last_line_at = started_at
    last_status = start
    last_change_at = started_at

    motion_seen = False
    stable_since = None

    while time.monotonic() - started_at < MOTION_TIMEOUT_S:
        now = time.monotonic()

        # If telemetry is quiet, explicitly request a fresh status.
        if now - last_line_at >= TELEMETRY_TIMEOUT_S:
            send_command(esp, "P")
            last_line_at = now

        line = esp.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        status = parse_status(line)
        if status is None:
            continue

        now = time.monotonic()
        last_line_at = now

        right_delta = status["right_pos"] - start["right_pos"]
        left_delta = status["left_pos"] - start["left_pos"]

        if (
            abs(right_delta) > 1.0
            or abs(left_delta) > 1.0
            or abs(status["right_vel"]) > VELOCITY_TOLERANCE
            or abs(status["left_vel"]) > VELOCITY_TOLERANCE
        ):
            motion_seen = True

        position_change = max(
            abs(status["right_pos"] - last_status["right_pos"]),
            abs(status["left_pos"] - last_status["left_pos"]),
        )

        velocities_low = (
            abs(status["right_vel"]) <= VELOCITY_TOLERANCE
            and abs(status["left_vel"]) <= VELOCITY_TOLERANCE
        )

        if position_change > POSITION_STABILITY_DEG:
            last_change_at = now
            stable_since = None
        elif (
            motion_seen
            and velocities_low
            and now - last_change_at >= STABLE_TIME_S
        ):
            if stable_since is None:
                stable_since = now

            if now - stable_since >= STABLE_TIME_S:
                print("Motion appears settled. Verifying with P...")

                # Explicit final status request.
                final_status = read_initial_status(esp)

                final_change = max(
                    abs(final_status["right_pos"] - status["right_pos"]),
                    abs(final_status["left_pos"] - status["left_pos"]),
                )

                final_velocities_low = (
                    abs(final_status["right_vel"]) <= VELOCITY_TOLERANCE
                    and abs(final_status["left_vel"]) <= VELOCITY_TOLERANCE
                )

                if (
                    final_change <= POSITION_STABILITY_DEG
                    and final_velocities_low
                ):
                    print(
                        "Motion complete according to ESP32 feedback.",
                        f"\nFinal status: {final_status}"
                    )
                    return

                # Motion or readings changed; continue monitoring.
                last_status = final_status
                last_change_at = time.monotonic()
                stable_since = None
                continue
        else:
            stable_since = None

        print(
            f"Right: {right_delta:+.1f} deg | "
            f"Left: {left_delta:+.1f} deg | "
            f"Velocity: R={status['right_vel']:+.1f}, "
            f"L={status['left_vel']:+.1f} deg/s"
        )

        last_status = status

    send_command(esp, "S")
    raise TimeoutError(
        "Motion did not settle within the safety timeout. "
        "A ramped stop was requested."
    )


def move_wheels(esp, right_angle, left_angle, speed=SPEED_DEG_S):
    # 1. Record the starting pose.
    start = read_initial_status(esp)
    print("Starting positions:", start)

    # 2. Send relative position commands to the firmware.
    send_command(esp, f"M1 {right_angle:.3f} {speed:.3f}")
    send_command(esp, f"M2 {left_angle:.3f} {speed:.3f}")

    # 3. Monitor the actual reported movement.
    wait_for_motion_to_settle(esp, start)


def forward(esp, distance_cm):
    angle = distance_cm / WHEEL_RADIUS_CM * 180.0 / math.pi
    print(f"Moving forward {distance_cm:g} cm")
    move_wheels(esp, angle, angle)


def backward(esp, distance_cm):
    angle = -distance_cm / WHEEL_RADIUS_CM * 180.0 / math.pi
    print(f"Moving backward {distance_cm:g} cm")
    move_wheels(esp, angle, angle)


def turn(esp, angle_deg):
    wheel_angle = (
        angle_deg * TRACK_WIDTH_CM / (2.0 * WHEEL_RADIUS_CM)
    )

    print(f"Turning {angle_deg:g} degrees")

    # Positive angle = counter-clockwise.
    move_wheels(esp, -wheel_angle, wheel_angle)


def main():
    with serial.Serial(PORT, BAUD, timeout=0.1) as esp:
        time.sleep(2.0)
        esp.reset_input_buffer()

        send_command(
            esp, f"A {ACCEL_DEG_S2:.1f} {DECEL_DEG_S2:.1f}"
        )

        print("Initial status:", read_initial_status(esp))
        print("\nCommands: forward 10 | backward 10 | turn 90 | turn -90 | quit")

        while True:
            parts = input("\nroomba> ").strip().lower().split()

            if not parts:
                continue

            if parts[0] in ("quit", "exit"):
                break

            try:
                if len(parts) != 2:
                    print("Usage: forward 10, backward 10, or turn 90")
                    continue

                value = float(parts[1])

                if not math.isfinite(value):
                    print("Value must be finite.")
                elif parts[0] == "forward" and value > 0:
                    forward(esp, value)
                elif parts[0] == "backward" and value > 0:
                    backward(esp, value)
                elif parts[0] == "turn":
                    turn(esp, value)
                else:
                    print("Invalid command or distance.")

            except (ValueError, TimeoutError, serial.SerialException) as exc:
                print(f"Motion/communication error: {exc}")
                print("Inspect the robot before continuing.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted. Stop the robot safely before restarting.")