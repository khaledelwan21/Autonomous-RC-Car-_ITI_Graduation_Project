# Line Follower Robot

A two-wheeled robot that follows a line autonomously, avoids obstacles with an
ultrasonic sensor, and can also be driven manually over WiFi (PS4 controller
via ROS 2). Live wheel speed and obstacle status can be viewed wirelessly on
a laptop GUI while the robot drives untethered.

![Robot](robot.jpg)

## How it works

```
 ┌─────────────┐   UART (E:/S: protocol)   ┌──────────────┐   UDP (WiFi)   ┌──────────────┐
 │   AVR MCU   │ ─────────────────────────▶ │    ESP32     │ ─────────────▶ │   Laptop     │
 │ (ATmega32)  │                            │              │                │  GUI / ROS2  │
 │             │                            │ - Motor PID  │ ◀───────────── │              │
 │ - 3x IR line│                            │ - Line follow│   UDP (cmd)    │              │
 │   sensors   │                            │ - Manual mode│                │              │
 │ - HC-SR04   │                            │ - Obstacle   │                │              │
 │   ultrasonic│                            │   stop       │                │              │
 └─────────────┘                            └──────────────┘                └──────────────┘
```

- The **AVR** reads 3 analog IR sensors, computes a line-position error, and
  reads an HC-SR04 ultrasonic sensor for obstacle detection. It sends
  `E:<error>` or `S:1` (obstacle) lines over UART to the ESP32.
- The **ESP32** runs the motor PID/PWM control loop, decides target wheel
  speed from either the line-follow error or manual UDP/Serial commands, and
  broadcasts live telemetry (`L:<rpm> R:<rpm> OBS:<0/1>`) over WiFi.
- The **laptop** can drive the robot manually (ROS 2 `wifi_bridge.py`,
  `/cmd_vel` + joystick button to toggle mode) and/or watch live speed and
  obstacle status wirelessly (`speed_display.py`).

## Repository structure

```
.
├── avr/
│   ├── main.h              # UART, ADC, ultrasonic function implementations
│   └── main.c              # main() - line-follow + obstacle-avoidance loop
├── esp32/
│   └── car_robot_ps4_final.ino   # motor control, line-follow, manual mode, telemetry
├── laptop/
│   ├── speed_display.py    # wireless GUI: live RPM + obstacle warning
│   ├── wifi_bridge.py      # ROS2 node: /cmd_vel + /joy -> UDP commands
│   └── robot.jpg           # robot photo used by speed_display.py
└── README.md
```

## Hardware

- **AVR (ATmega32)**
  - 3x analog IR line sensors -> `ADC0`, `ADC1`, `ADC2` (`PA0`-`PA2`)
  - HC-SR04 ultrasonic: `TRIG` -> `PD6`, `ECHO` -> `PB2`
  - UART -> ESP32 `Serial2` (`RX2` = GPIO16, `TX2` = GPIO17), 9600 baud
- **ESP32**
  - Motor driver: `IN1`/`IN2` (right, GPIO27/26), `IN3`/`IN4` (left, GPIO25/14), PWM via `ledc`
  - Wheel encoders: left `GPIO32`/`33`, right `GPIO19`/`21`
  - WiFi Access Point: SSID `CarRobotPS4`, password `robot1234`

## Firmware setup (PlatformIO)

**AVR** — put the header in `include/`, the source in `src/`:
```
avr_project/
├── platformio.ini
├── include/
│   └── main.h
└── src/
    └── main.c
```

**ESP32** — single `.ino` file works as-is in PlatformIO or Arduino IDE.

## Communication protocol (AVR → ESP32, UART2 @ 9600)

| Message | Meaning |
|---|---|
| `E:<int>\n` | Line position error (negative = drifted left, positive = drifted right) |
| `S:1\n` | Obstacle closer than `STOP_DISTANCE_CM` (10cm) - stop immediately |
| `E:ADC_TIMEOUT\n` | ADC read failed (check AVCC/AREF wiring) |

## Telemetry protocol (ESP32 → laptop, UDP broadcast, port 4211)

```
L:<left_rpm> R:<right_rpm> OBS:<0|1>
```
Broadcast ~10 times/sec to `192.168.4.255:4211`.

## Manual control protocol (laptop → ESP32, UDP, port 4210)

```
<left_pwm>,<right_pwm>    # e.g. "120,-80"
MODE:TOGGLE                # switch between MANUAL and LINE_FOLLOW
```

## Running the laptop apps

```bash
# Wireless speed/obstacle display (keep robot.jpg in the same folder)
pip install pillow --break-system-packages
python3 speed_display.py

# ROS2 bridge for manual PS4 control
ros2 run <your_package> wifi_bridge.py
```

Connect the laptop's WiFi to `CarRobotPS4` before running either.

## Notes

- Obstacle stop (`OBS`) takes priority over both manual and line-follow
  driving - the robot stops immediately regardless of mode when the AVR
  reports something closer than 10cm.
- The ESP32 also accepts manual commands over USB Serial (115200 baud) as a
  wired fallback to WiFi.
