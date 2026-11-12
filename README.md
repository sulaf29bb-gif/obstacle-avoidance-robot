# Raspberry Pi Obstacle Avoidance Robot

A real-time obstacle avoidance controller for a small two-wheeled robot built on a Raspberry Pi. An ultrasonic sensor watches the path ahead; the robot slows down when something is near, stops immediately when something is too close, backs away, pivots to a new heading, and carries on.

## Features

- **Single control loop** owns the motors and the sensor, so there are no race conditions between "speed up" and "slow down" logic.
- **Immediate emergency stop** (no ramp-down) when an obstacle is inside the stop distance.
- **Real pivot turns** (wheels spin in opposite directions). With optional left/right sensors it turns toward the clearer side; otherwise it picks a random direction.
- **Filtered distance readings** (median of 3) with a clear distinction between "nothing in range" and "sensor fault".
- **Fail-safe state**: if the sensor stops returning data, the robot is boxed in, or it cannot back away, it halts and waits before trying again.
- **Motor driver fault handling**: reads the driver's fault pin and pulses RESET.
- **Live console dashboard** and a log file (`robot_log.txt`).
- **Configurable**: pins, distances, speeds and timings are constants at the top of `robot.py`.

## Behaviour

```
                    d < STOP_CM
   CRUISE  ─────────────────────────►  BACKING ──(d ≥ CLEAR_CM)──► TURNING
     ▲  │ d < WARN_CM                     │                           │
     │  ▼                                 │ timeout                   │ turn done
     │ WARNING (slow)                     ▼                           │
     │                                FAILSAFE ◄── no sensor data     │
     │                                    │ after RECOVERY_S          │
     └────────────────────────────────────┴───────────────────────────┘
```

| Distance to obstacle | Action |
|---|---|
| ≥ 40 cm | Cruise at 20 cm/s |
| 20 – 40 cm | Slow to 10 cm/s |
| < 20 cm | Stop instantly, reverse until ≥ 30 cm, pivot, resume |

## Hardware

- Raspberry Pi (intended for: Pi 3/4/Zero 2; Pi 5 needs `PWM_CHIP = 2`)
- Dual DC motor driver with DIR/ENABLE per side plus SLEEP, RESET, DECAY and nFAULT pins
- Two DC motors
- HC-SR04 (or compatible) ultrasonic sensor for the front
- Optional: two more ultrasonic sensors (left and right) for smarter turning
- Resistors for a voltage divider (or a level shifter) on the sensor's ECHO line
- Battery pack suitable for your motors, with the grounds of the Pi, driver and sensor connected together

### Default wiring (BCM numbers)

| Signal | GPIO | Notes |
|---|---|---|
| ENBL | 12 | Left bridge enable |
| DIRL | 20 | Left direction |
| ENBR | 21 | Right bridge enable |
| DIRR | 26 | Right direction |
| SLEEP | 8 | Driver sleep (held high) |
| RESET | 7 | Driver reset (held high, pulsed on fault) |
| DECAY | 16 | Decay mode |
| FAULT | 25 | Driver nFAULT, active low |
| TRIG | 27 | Ultrasonic trigger |
| ECHO | 17 | Ultrasonic echo, **needs a voltage divider** |
| TRIG/ECHO left, right | not set | Optional. Set `PIN_TRIG_LEFT`, `PIN_ECHO_LEFT`, `PIN_TRIG_RIGHT`, `PIN_ECHO_RIGHT` in `robot.py` to enable. Echo pins need dividers too. |
| PWM0 | GPIO 18 | Left speed input on the driver |
| PWM1 | GPIO 19 | Right speed input on the driver |


## Installation

```bash
git clone https://github.com/sulaf29bb-gif/obstacle-avoidance-robot.git
cd obstacle-avoidance-robot
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

Enable hardware PWM by adding this line to `/boot/firmware/config.txt` (older systems: `/boot/config.txt`) and rebooting:

```
dtoverlay=pwm-2chan
```

## Usage

Put the robot on blocks (wheels off the ground) for the first run, then:

```bash
python3 robot.py
```

Press `Ctrl+C` to stop. The motors are disabled on exit.

Example dashboard output:

```
[cruise  ] front=  150 cm  speed=20.0 cm/s  PWM L= 100 R= 100
[warning ] front=   35 cm  speed=14.2 cm/s  PWM L=  71 R=  71
[backing ] front=   18 cm  speed= 0.0 cm/s  PWM L= -50 R= -50
```

## Calibration and tuning

The speed values are **estimates**: there are no wheel encoders, so `SPEED_TABLE` maps a nominal speed (cm/s) to a PWM value that you must measure yourself.

1. Run the robot at a fixed PWM over a known distance (for example 1 m) and time it.
2. Compute cm/s and update `SPEED_TABLE` with `(speed, pwm)` pairs.
3. If a wheel spins the wrong way, flip `LEFT_FORWARD_LEVEL` / `RIGHT_FORWARD_LEVEL`.
4. If the robot sometimes hits obstacles, increase `STOP_CM` or lower `CRUISE_SPEED`.
5. If the driver always reports a fault, set `FAULT_ACTIVE_LOW = False`.

## Limitations

- Side sensors (if fitted) are read only when choosing a turn. The robot has nothing watching the rear while reversing, so keep reversing speeds low.
- Ultrasonic sensors struggle with soft, angled or very thin objects (fabric, chair legs, steep slopes).
- Distance readings use Python busy-wait timing on a non-realtime OS, so expect a few centimetres of jitter. For better accuracy, consider `gpiozero.DistanceSensor` or a pigpio/lgpio-based reader.
- The speed estimate is open-loop (no encoders).

## Safety

Test with the wheels off the ground first. Keep a physical power switch within reach and never leave a running robot unattended near stairs or people.

## Project structure

```
.
├── robot.py            # controller (hardware, state machine, dashboard)
├── Requirements.txt
├── README.md
├── LICENSE
└── .gitignore
```

## License

MIT LISENCE (SEE LICENSE FILE)
