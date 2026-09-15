# PS4-Controlled & Autonomous Line-Following Robot

A two-wheel differential-drive robot built on **ROS 2 Jazzy** and an **ESP32**, controllable either manually with a PS4 controller or autonomously as a line follower with obstacle detection — switchable on the fly with a single controller button.

![Robot](./assets/robot.jpeg)

## Features

- **Manual control** via a PS4 (DualShock 4) controller, using ROS 2's `joy` and `teleop_twist_joy` packages
- **Dual link options** between the laptop/Pi and the ESP32: WiFi (UDP, ESP32 as its own access point) or USB Serial — both accepted simultaneously
- **Closed-loop speed control** on both wheels using quadrature encoders and a PID controller, so both motors track the same RPM regardless of manufacturing differences
- **Smooth acceleration ramping** to avoid sudden jerks/wheelies when speed changes
- **Autonomous line following** using 3 analog IR sensors read by a separate AVR microcontroller, which computes a weighted-average position error and streams it to the ESP32 over UART
- **Obstacle detection**: the AVR reports a stop condition that immediately halts the robot regardless of the line-following logic
- **One-button mode switch** (Manual ⇄ Line Follow) from the PS4 controller, or via a `MODE:TOGGLE` command over WiFi/Serial
- **Safety watchdogs**: the robot stops automatically if the WiFi/Serial link drops (manual mode) or if the AVR sensor link drops (line-follow mode)
- **Live wireless speed dashboard** showing both wheels' RPM in real time

![Speed dashboard](./assets/speed_dashboard.png)

## Hardware

| Component | Role |
|---|---|
| ESP32 DevKit | Main controller: motor PID, WiFi AP, mode switching |
| AVR microcontroller (8 MHz, bare-metal AVR-GCC) | Reads 3 analog IR sensors, computes line-position error, sends it to the ESP32 over UART |
| 2× GA25-370 DC gear motors (12V, with encoders) | Drive wheels |
| L293D motor driver | Drives both motors, PWM applied directly on the IN pins (ENA/ENB tied to 5V) |
| PS4 (DualShock 4) controller | Manual driving input, connects to the laptop/Pi over Bluetooth |
| Laptop / Raspberry Pi 4 running ROS 2 Jazzy | Runs `joy_node`, `teleop_twist_joy`, and the bridge node to the ESP32 |

## System Architecture

```
PS4 Controller (Bluetooth)
        |
    joy_node  ->  /joy
        |
teleop_twist_joy  ->  /cmd_vel
        |
  bridge node (WiFi UDP or USB Serial)
        |
        v
      ESP32  <-- UART --  AVR (3x IR sensors)
        |
   PID + Encoders
        |
   L293D  ->  DC Motors
```

- In **MANUAL** mode, the ESP32 drives the wheels according to speed commands received over WiFi/Serial.
- In **LINE_FOLLOW** mode, the ESP32 ignores manual commands and instead steers based on the position error streamed continuously by the AVR.
- Both modes share the same underlying PID + encoder feedback loop for actually reaching the requested wheel speed.

## Repository Contents

| File | Description |
|---|---|
| `main.cpp` | ESP32 firmware: WiFi AP, UDP + Serial command handling, PID motor control, encoder reading, line-follow mode, obstacle stop |
| `avr_line_sensor.c` | Bare-metal AVR-GCC firmware: reads 3 analog IR sensors, computes the line-position error, sends it over UART |
| `wifi_bridge.py` | ROS 2 node: converts `/cmd_vel` to wheel speeds and sends them to the ESP32 over WiFi UDP; also forwards a mode-toggle command from a controller button |
| `serial_bridge.py` | Same as above, but sends over USB Serial instead of WiFi |
| `ps4_config.yaml` | `teleop_twist_joy` configuration mapping the PS4 controller's axes/buttons |

## Setup

1. Flash `avr_line_sensor.c` to the AVR board (built for an 8 MHz clock).
2. Flash `main.cpp` to the ESP32 via PlatformIO/Arduino IDE.
3. Wire the AVR's UART TX to the ESP32's UART2 RX (GPIO 16), with a common ground.
4. On the laptop/Pi, install ROS 2 Jazzy and the required packages:
   ```bash
   sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy python3-serial
   ```
5. Pair the PS4 controller over Bluetooth.

## Running It

```bash
# Terminal 1
ros2 run joy joy_node

# Terminal 2
ros2 run teleop_twist_joy teleop_node --ros-args --params-file ps4_config.yaml

# Terminal 3 (choose one)
python3 wifi_bridge.py     # connect to the "CarRobotPS4" WiFi network first
# or
python3 serial_bridge.py   # connect the ESP32 over USB
```

Drive the robot manually onto the line, then press the assigned controller button to switch into autonomous line-following mode. Press it again to take back manual control at any time.

## Tuning Notes

- `Kp`, `Ki`, `Kd` in `main.cpp` control the wheel-speed PID loop.
- `LINE_KP` and `LINE_BASE_RPM` control how aggressively and how fast the robot follows the line.
- `MAX_TARGET_RPM` should match the motors' real achievable speed under load, not just their no-load datasheet rating.

## Possible Next Steps

- Move the WiFi bridge from a laptop to an onboard Raspberry Pi for a fully self-contained robot
- Add a physical on/off switch for line-follow mode as a hardware-only fallback
- Migrate the ESP32 link to ESP-NOW for longer, more reliable range than WiFi AP mode
