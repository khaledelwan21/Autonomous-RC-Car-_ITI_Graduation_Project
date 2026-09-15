#!/usr/bin/env python3
"""
Live speed display for the line-follower robot - wireless version.

Receives "L:<rpm> R:<rpm>" UDP broadcast packets from the ESP32 over WiFi
(no cable needed). Before running this:
  1. Connect your laptop's WiFi to the robot's access point (default SSID
     "LineFollowerBot", password "robot1234" - check your ESP32 code for
     the actual values if you changed them).
  2. Run this script - it listens on the same UDP port the ESP32 sends to.

Usage:
    python3 speed_display.py            # uses default port below
    python3 speed_display.py 5005       # or pass a different port
"""

import re
import socket
import sys
import threading
import tkinter as tk

from PIL import Image, ImageTk

DEFAULT_PORT = 4211

# Keep robot.jpg in the same folder as this script.
ROBOT_IMAGE_PATH = "robot.jpg"
ROBOT_IMAGE_SIZE = (380, 285)

# Matches "L:42.3 R:40.1 OBS:0" (new firmware) or plain "L:42.3 R:40.1"
# (older firmware without obstacle telemetry) - OBS is optional so this
# still works either way instead of silently failing to match.
RPM_PATTERN = re.compile(r"L:\s*(-?\d+\.?\d*)\s*R:\s*(-?\d+\.?\d*)(?:\s*OBS:\s*([01]))?")


class SpeedDisplay:
    def __init__(self, root, port):
        self.root = root
        self.root.title("Robot Speed (Wireless)")
        self.root.geometry("480x560")
        self.root.configure(bg="#111111")

        self.status_var = tk.StringVar(value=f"Listening on UDP port {port}...")
        tk.Label(
            root, textvariable=self.status_var, bg="#111111", fg="#888888",
            font=("Helvetica", 11)
        ).pack(pady=(10, 0))

        # Real robot photo. A red-tinted version is precomputed once and
        # swapped in when an obstacle is reported, instead of loading/
        # blending the image again on every update.
        self.normal_photo, self.warning_photo = self._load_robot_images()
        self.robot_canvas = tk.Canvas(root, width=ROBOT_IMAGE_SIZE[0],
                                       height=ROBOT_IMAGE_SIZE[1], bg="#111111",
                                       highlightthickness=0)
        self.robot_canvas.pack(pady=(10, 0))
        self.robot_image_id = self.robot_canvas.create_image(
            0, 0, anchor="nw", image=self.normal_photo
        )

        self.obstacle_var = tk.StringVar(value="")
        tk.Label(root, textvariable=self.obstacle_var, bg="#111111", fg="#ff4d4d",
                 font=("Helvetica", 13, "bold")).pack(pady=(2, 0))

        row = tk.Frame(root, bg="#111111")
        row.pack(expand=True)

        left_col = tk.Frame(row, bg="#111111")
        left_col.grid(row=0, column=0, padx=30)
        tk.Label(left_col, text="LEFT", bg="#111111", fg="#666666",
                 font=("Helvetica", 14)).pack()
        self.left_var = tk.StringVar(value="--")
        tk.Label(left_col, textvariable=self.left_var, bg="#111111", fg="#00d97e",
                 font=("Helvetica", 48, "bold")).pack()
        tk.Label(left_col, text="RPM", bg="#111111", fg="#666666",
                 font=("Helvetica", 12)).pack()

        right_col = tk.Frame(row, bg="#111111")
        right_col.grid(row=0, column=1, padx=30)
        tk.Label(right_col, text="RIGHT", bg="#111111", fg="#666666",
                 font=("Helvetica", 14)).pack()
        self.right_var = tk.StringVar(value="--")
        tk.Label(right_col, textvariable=self.right_var, bg="#111111", fg="#00d97e",
                 font=("Helvetica", 48, "bold")).pack()
        tk.Label(right_col, text="RPM", bg="#111111", fg="#666666",
                 font=("Helvetica", 12)).pack()

        self.port = port
        self.running = True

        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _load_robot_images(self):
        """Loads robot.jpg and precomputes a red-tinted 'warning' version.
        Falls back to a plain grey box if the image file isn't found next
        to this script, so the app still runs (just without the photo)."""
        try:
            base_image = Image.open(ROBOT_IMAGE_PATH).convert("RGB").resize(ROBOT_IMAGE_SIZE)
        except FileNotFoundError:
            base_image = Image.new("RGB", ROBOT_IMAGE_SIZE, "#333333")

        normal_photo = ImageTk.PhotoImage(base_image)

        red_overlay = Image.new("RGB", ROBOT_IMAGE_SIZE, "#ff0000")
        warning_image = Image.blend(base_image, red_overlay, alpha=0.45)
        warning_photo = ImageTk.PhotoImage(warning_image)

        return normal_photo, warning_photo

    def _read_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.port))
        except OSError as e:
            self.status_var.set(f"Could not bind UDP port {self.port}: {e}")
            return

        sock.settimeout(1.0)
        self.sock = sock

        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            line = data.decode("utf-8", errors="replace").strip()
            match = RPM_PATTERN.search(line)
            if match:
                left_rpm = float(match.group(1))
                right_rpm = float(match.group(2))
                obstacle = match.group(3) == "1"  # None (no OBS field) counts as False
                self.status_var.set(f"Receiving from {addr[0]}")
                # Update from the background thread using after() to stay
                # thread-safe with Tkinter.
                self.root.after(0, self._update_display, left_rpm, right_rpm, obstacle)

        sock.close()

    def _update_display(self, left_rpm, right_rpm, obstacle):
        self.left_var.set(f"{left_rpm:.1f}")
        self.right_var.set(f"{right_rpm:.1f}")

        if obstacle:
            self.robot_canvas.itemconfig(self.robot_image_id, image=self.warning_photo)
            self.obstacle_var.set("OBSTACLE DETECTED")
        else:
            self.robot_canvas.itemconfig(self.robot_image_id, image=self.normal_photo)
            self.obstacle_var.set("")

    def _on_close(self):
        self.running = False
        self.root.destroy()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT

    root = tk.Tk()
    SpeedDisplay(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
