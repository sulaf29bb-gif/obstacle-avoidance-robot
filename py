#!/usr/bin/env python3
"""
Real-time obstacle avoidance for a two-wheeled Raspberry Pi robot.

Hardware assumed:
  * Raspberry Pi (BCM pin numbering)
  * Dual H-bridge motor driver with per-side DIR/ENABLE pins plus SLEEP, RESET,
    DECAY and an open-drain nFAULT output
  * Two hardware PWM channels feeding the driver's speed inputs
  * One HC-SR04-style ultrasonic sensor facing forward
  * Optionally two more (left/right) used to pick the clearer turn direction

Design: ONE control loop owns the motors and the sensor. A separate dashboard
thread only reads a telemetry snapshot, so there are no races over the motors.

All tunable values live in the CONFIG section below.
"""

import logging
import random
import signal
import statistics
import threading
import time
from collections import deque

from gpiozero import DigitalInputDevice, DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM

# =============================================================================
# CONFIG
# =============================================================================

# --- GPIO pins (BCM numbering) ---
PIN_ENBL = 12
PIN_DIRL = 20
PIN_ENBR = 21
PIN_DIRR = 26
PIN_SLEEP = 8
PIN_RESET = 7
PIN_DECAY = 16
PIN_FAULT = 25
PIN_TRIG = 27
PIN_ECHO = 17   # MUST be level-shifted / divided down to 3.3 V (see README)

# Optional side sensors, read only when choosing which way to turn.
# None = not fitted (the robot then turns in a random direction).
# Their ECHO pins ALSO need a 5 V -> 3.3 V divider.
PIN_TRIG_LEFT = None
PIN_ECHO_LEFT = None
PIN_TRIG_RIGHT = None
PIN_ECHO_RIGHT = None
SIDE_MARGIN_CM = 5          # sides must differ by more than this to prefer one

# --- PWM ---
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
PWM_CHIP = 0            # Raspberry Pi 5 uses chip 2
PWM_HZ = 1000

# --- Motor wiring ---
# DIR pin level that makes each wheel roll forward. Flip if a wheel runs backward.
LEFT_FORWARD_LEVEL = True
RIGHT_FORWARD_LEVEL = True
FAULT_ACTIVE_LOW = True  # most drivers expose nFAULT (active low)

# --- Speed calibration: (speed in cm/s, PWM 0-255). MEASURE THESE on your robot. ---
SPEED_TABLE = [(0, 0), (10, 50), (20, 100), (30, 150)]
CRUISE_SPEED = 20       # cm/s
SLOW_SPEED = 10         # cm/s, used in the warning zone
ACCEL = 30.0            # cm/s^2
DECEL = 60.0            # cm/s^2
BACK_SPEED = 10         # cm/s while reversing
TURN_SPEED = 10         # cm/s per wheel while pivoting

# --- Distances (cm) ---
STOP_CM = 20            # inside this: stop immediately and evade
WARN_CM = 40            # inside this: slow down
CLEAR_CM = 30           # reverse until at least this far from the obstacle
MAX_RANGE_CM = 400

# --- Timing ---
LOOP_PERIOD = 0.05      # target control loop period (s); sensor reads may stretch it
BACK_TIMEOUT_S = 3.0    # give up reversing after this long
TURN_TIME_RANGE = (0.5, 0.9)   # random pivot duration (s)
RECOVERY_S = 2.0        # fail-safe hold time
MAX_AVOID_ATTEMPTS = 4  # consecutive evasions before declaring "boxed in"
ATTEMPT_RESET_S = 3.0   # quiet time after which the attempt counter resets
MAX_SENSOR_FAILS = 10   # consecutive failed reads before fail-safe

# --- Ultrasonic ---
ECHO_START_TIMEOUT = 0.02   # wait for echo pulse to begin (no start = sensor fault)
ECHO_END_TIMEOUT = 0.03     # echo pulse longer than this = nothing in range
SAMPLES_PER_READ = 3        # median filter size

LOG_FILE = "robot_log.txt"

# =============================================================================

logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format="%(asctime)s - %(message)s",
)

# States
CRUISE, WARNING, BACKING, TURNING, FAILSAFE = (
    "cruise", "warning", "backing", "turning", "failsafe"
)


def speed_to_pwm(speed):
    """Linearly interpolate PWM (0-255) from the calibrated speed table."""
    if speed <= 0:
        return 0
    for (s1, p1), (s2, p2) in zip(SPEED_TABLE, SPEED_TABLE[1:]):
        if speed <= s2:
            return int(round(p1 + (p2 - p1) * (speed - s1) / (s2 - s1)))
    return SPEED_TABLE[-1][1]


class MotorDriver:
    """Owns all motor-driver pins and the two hardware PWM channels."""

    def __init__(self):
        self.enbl = DigitalOutputDevice(PIN_ENBL)
        self.dirl = DigitalOutputDevice(PIN_DIRL)
        self.enbr = DigitalOutputDevice(PIN_ENBR)
        self.dirr = DigitalOutputDevice(PIN_DIRR)
        self.sleep = DigitalOutputDevice(PIN_SLEEP)
        self.reset = DigitalOutputDevice(PIN_RESET)
        self.decay = DigitalOutputDevice(PIN_DECAY)
        # pull_up=True -> is_active is True when the pin is pulled LOW (nFAULT)
        self.fault = DigitalInputDevice(PIN_FAULT, pull_up=FAULT_ACTIVE_LOW)

        self.left_pwm = 0
        self.right_pwm = 0
        self._last_fault_reset = 0.0

        self.enbl.off()
        self.enbr.off()
        self.sleep.on()
        self.reset.on()
        self.decay.off()

        self.pwm_l = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_HZ, chip=PWM_CHIP)
        self.pwm_r = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_HZ, chip=PWM_CHIP)
        self.pwm_l.start(0)
        self.pwm_r.start(0)

    def _check_fault(self):
        if self.fault.is_active and time.monotonic() - self._last_fault_reset > 1.0:
            logging.info("Driver fault detected - pulsing RESET")
            self.reset.off()
            time.sleep(0.05)
            self.reset.on()
            self._last_fault_reset = time.monotonic()

    def drive(self, left, right):
        """
        Drive each wheel with a signed PWM value in [-255, 255].
        Positive = forward, negative = backward. Opposite signs = pivot turn.
        """
        self._check_fault()
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))

        self.dirl.value = (left >= 0) == LEFT_FORWARD_LEVEL
        self.dirr.value = (right >= 0) == RIGHT_FORWARD_LEVEL
        self.sleep.on()
        self.enbl.on()
        self.enbr.on()
        self.pwm_l.change_duty_cycle(abs(left) / 255 * 100)
        self.pwm_r.change_duty_cycle(abs(right) / 255 * 100)
        self.left_pwm, self.right_pwm = left, right

    def stop(self):
        """Immediate stop: zero duty and disable both bridges."""
        try:
            self.pwm_l.change_duty_cycle(0)
            self.pwm_r.change_duty_cycle(0)
            self.enbl.off()
            self.enbr.off()
        except Exception as exc:  # keep shutting down even if PWM errors
            logging.info(f"Stop error: {exc}")
        self.left_pwm = self.right_pwm = 0

    def close(self):
        self.stop()
        for pwm in (self.pwm_l, self.pwm_r):
            try:
                pwm.stop()
            except Exception as exc:
                logging.info(f"PWM close error: {exc}")
        self.sleep.off()


class UltrasonicSensor:
    """HC-SR04-style distance sensor with timeouts and a median filter."""

    def __init__(self, trig_pin=PIN_TRIG, echo_pin=PIN_ECHO):
        self.trig = DigitalOutputDevice(trig_pin)
        self.echo = DigitalInputDevice(echo_pin)

    def _measure_once(self):
        """Return distance in cm, MAX_RANGE_CM if nothing in range, None on sensor fault."""
        self.trig.off()
        time.sleep(0.0002)
        self.trig.on()
        time.sleep(0.00001)
        self.trig.off()

        deadline = time.perf_counter() + ECHO_START_TIMEOUT
        while not self.echo.is_active:
            if time.perf_counter() > deadline:
                return None  # echo never started: wiring/sensor problem
        start = time.perf_counter()

        deadline = start + ECHO_END_TIMEOUT
        while self.echo.is_active:
            if time.perf_counter() > deadline:
                return MAX_RANGE_CM  # pulse too long: nothing in range
        duration = time.perf_counter() - start

        return max(0.0, min(MAX_RANGE_CM, duration * 17150))

    def read(self):
        """Median of several samples; None if every sample failed."""
        samples = []
        for _ in range(SAMPLES_PER_READ):
            value = self._measure_once()
            if value is not None:
                samples.append(value)
            time.sleep(0.03)  # let echoes die out between pings
        return statistics.median(samples) if samples else None

    def close(self):
        self.trig.close()
        self.echo.close()


class Telemetry:
    """Thread-safe status store used by the dashboard."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "state": "idle", "distance": None, "speed": 0.0,
            "left_pwm": 0, "right_pwm": 0,
        }
        self.events = deque(maxlen=10)

    def update(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)

    def snapshot(self):
        with self._lock:
            return dict(self._data)

    def event(self, message):
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self.events.append(f"{stamp} - {message}")
        logging.info(message)


def dashboard_loop(telemetry, stop_event):
    while not stop_event.wait(1.0):
        s = telemetry.snapshot()
        dist = "---" if s["distance"] is None else f"{s['distance']:5.0f}"
        print(
            f"[{s['state']:<8}] front={dist} cm  "
            f"speed={s['speed']:4.1f} cm/s  "
            f"PWM L={s['left_pwm']:>4} R={s['right_pwm']:>4}"
        )


class Robot:
    """Single control loop / state machine for obstacle avoidance."""

    def __init__(self, driver, sonar, telemetry, left_sonar=None, right_sonar=None):
        self.driver = driver
        self.sonar = sonar
        self.left_sonar = left_sonar
        self.right_sonar = right_sonar
        self.telemetry = telemetry

        self.state = CRUISE
        self.state_since = time.monotonic()
        self.speed = 0.0                # estimated forward speed, cm/s
        self.last_distance = None
        self.sensor_fails = 0
        self.attempts = 0
        self.last_obstacle_time = 0.0
        self.turn_until = 0.0
        self.turn_sign = 1
        self.dt = LOOP_PERIOD
        self._stop_evt = threading.Event()

    # ---- helpers ------------------------------------------------------------
    def request_stop(self, *_):
        self._stop_evt.set()

    def _enter(self, state, now, reason=None):
        if state != self.state:
            self.state = state
            self.state_since = now
            if reason:
                self.telemetry.event(f"{state.upper()}: {reason}")

    def _halt(self):
        self.driver.stop()
        self.speed = 0.0

    def _ramp_to(self, target):
        rate = ACCEL if target > self.speed else DECEL
        step = rate * self.dt
        if self.speed < target:
            self.speed = min(target, self.speed + step)
        else:
            self.speed = max(target, self.speed - step)
        pwm = speed_to_pwm(self.speed)
        if pwm == 0:
            self.driver.stop()
        else:
            self.driver.drive(pwm, pwm)

    def _choose_turn(self):
        """Return +1 (pivot right) or -1 (pivot left), preferring the clearer side."""
        if self.left_sonar and self.right_sonar:
            left, right = self.left_sonar.read(), self.right_sonar.read()
            if left is not None and right is not None and abs(left - right) > SIDE_MARGIN_CM:
                self.telemetry.event(f"side clearance L={left:.0f} R={right:.0f} cm")
                return -1 if left > right else 1
        return random.choice((-1, 1))

    def _read_sensor(self):
        """Filtered distance, or None if there is no trustworthy data."""
        d = self.sonar.read()
        if d is None:
            self.sensor_fails += 1
            if self.sensor_fails >= MAX_SENSOR_FAILS:
                return None
            return self.last_distance  # brief glitch: reuse last good value
        self.sensor_fails = 0
        self.last_distance = d
        return d

    # ---- state machine ------------------------------------------------------
    def _step(self, d, now):
        if d is None:
            if self.state != FAILSAFE:
                self._enter(FAILSAFE, now, "no valid distance readings")
            self._halt()
            return

        if self.state == FAILSAFE:
            self._halt()
            if now - self.state_since >= RECOVERY_S:
                self._enter(CRUISE, now, "recovered")
            return

        if self.state in (CRUISE, WARNING):
            if now - self.last_obstacle_time > ATTEMPT_RESET_S:
                self.attempts = 0

            if d < STOP_CM:
                self._halt()  # immediate stop, no ramp
                self.last_obstacle_time = now
                self.attempts += 1
                if self.attempts > MAX_AVOID_ATTEMPTS:
                    self.attempts = 0
                    self._enter(FAILSAFE, now, "boxed in, holding position")
                else:
                    self._enter(BACKING, now, f"obstacle at {d:.0f} cm")
                return

            if d < WARN_CM:
                self._enter(WARNING, now, f"slowing, obstacle at {d:.0f} cm")
                self._ramp_to(SLOW_SPEED)
            else:
                self._enter(CRUISE, now)
                self._ramp_to(CRUISE_SPEED)

        elif self.state == BACKING:
            self.last_obstacle_time = now
            if d >= CLEAR_CM:
                self._halt()
                self.turn_sign = self._choose_turn()
                self.turn_until = now + random.uniform(*TURN_TIME_RANGE)
                side = "right" if self.turn_sign > 0 else "left"
                self._enter(TURNING, now, f"pivoting {side}")
            elif now - self.state_since > BACK_TIMEOUT_S:
                self._halt()
                self._enter(FAILSAFE, now, "could not back away")
            else:
                pwm = speed_to_pwm(BACK_SPEED)
                self.driver.drive(-pwm, -pwm)

        elif self.state == TURNING:
            self.last_obstacle_time = now
            if now >= self.turn_until:
                self._halt()
                self._enter(CRUISE, now, "resuming navigation")
            else:
                pwm = speed_to_pwm(TURN_SPEED)
                # pivot in place: wheels spin in opposite directions
                self.driver.drive(self.turn_sign * pwm, -self.turn_sign * pwm)

    # ---- main loop ----------------------------------------------------------
    def run(self):
        threading.Thread(
            target=dashboard_loop, args=(self.telemetry, self._stop_evt), daemon=True
        ).start()
        self.telemetry.event("Robot started")
        last = time.monotonic()

        while not self._stop_evt.is_set():
            distance = self._read_sensor()
            now = time.monotonic()
            self.dt = max(0.001, now - last)
            last = now

            self._step(distance, now)
            self.telemetry.update(
                state=self.state, distance=distance, speed=self.speed,
                left_pwm=self.driver.left_pwm, right_pwm=self.driver.right_pwm,
            )
            self._stop_evt.wait(max(0.0, LOOP_PERIOD - (time.monotonic() - now)))

        self.telemetry.event("Robot stopped")


def main():
    driver = sonar = robot = None
    side_sonars = []
    try:
        driver = MotorDriver()
        sonar = UltrasonicSensor()
        left = right = None
        if PIN_TRIG_LEFT is not None and PIN_TRIG_RIGHT is not None:
            left = UltrasonicSensor(PIN_TRIG_LEFT, PIN_ECHO_LEFT)
            right = UltrasonicSensor(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT)
            side_sonars = [left, right]
        robot = Robot(driver, sonar, Telemetry(), left, right)
        signal.signal(signal.SIGTERM, robot.request_stop)  # clean exit under systemd
        robot.run()
    except KeyboardInterrupt:
        print("System stopping...")
    finally:
        if robot:
            robot.request_stop()
        if driver:
            driver.close()
        if sonar:
            sonar.close()
        for side in side_sonars:
            side.close()


if __name__ == "__main__":
    main()
