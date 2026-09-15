#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import time

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200

MAX_LINEAR = 0.5    # must match scale_linear.x in ps4_config.yaml
MAX_ANGULAR = 1.0   # must match scale_angular.yaw in ps4_config.yaml
MAX_PWM = 255

MODE_TOGGLE_BUTTON = 2  # change to whatever button you want to use


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', SERIAL_PORT)
        port = self.get_parameter('port').get_parameter_value().string_value

        self.current_left = 0
        self.current_right = 0
        self.prev_toggle_button_state = 0

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # wait for ESP32 to reset after opening the port
            self.get_logger().info(f'Connected to {port} at {BAUD_RATE} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Resend the current command on a fixed schedule, even if it hasn't
        # changed, so the ESP32's watchdog never thinks the link is dead
        # just because the joystick is being held steady.
        self.timer = self.create_timer(0.1, self.send_periodic)

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        self.current_left, self.current_right = self.twist_to_wheel_speeds(linear, angular)

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= MODE_TOGGLE_BUTTON:
            return

        current_state = msg.buttons[MODE_TOGGLE_BUTTON]

        # Only send on the rising edge (button just pressed), not on every
        # frame the button happens to be held down.
        if current_state == 1 and self.prev_toggle_button_state == 0:
            self.send_mode_toggle()

        self.prev_toggle_button_state = current_state

    def send_periodic(self):
        self.send_command(self.current_left, self.current_right)

    def twist_to_wheel_speeds(self, linear, angular):
        norm_linear = clamp(linear / MAX_LINEAR, -1.0, 1.0)
        norm_angular = clamp(angular / MAX_ANGULAR, -1.0, 1.0)

        left = clamp(norm_linear - norm_angular, -1.0, 1.0)
        right = clamp(norm_linear + norm_angular, -1.0, 1.0)

        left_pwm = int(left * MAX_PWM)
        right_pwm = int(right * MAX_PWM)

        return left_pwm, right_pwm

    def send_command(self, left_pwm, right_pwm):
        line = f'{left_pwm},{right_pwm}\n'
        try:
            self.ser.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {e}')

    def send_mode_toggle(self):
        try:
            self.ser.write(b'MODE:TOGGLE\n')
            self.get_logger().info('Sent mode toggle command')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send mode toggle: {e}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.write(b'0,0\n')
            self.ser.close()
        super().destroy_node()


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
