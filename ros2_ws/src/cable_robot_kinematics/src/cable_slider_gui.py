#!/usr/bin/env python3
# Copyright 2026 Alba López del Águila
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class CableSliderGui(Node):
    def __init__(self) -> None:
        super().__init__("cable_slider_gui")
        self.declare_parameter("slider_min", 0.0)
        self.declare_parameter("slider_max", 0.15)
        self.declare_parameter("slider_resolution", 0.005)

        self._publisher = self.create_publisher(JointState, "manual_cable_joint_commands", 10)
        self._pose_publisher = self.create_publisher(PoseStamped, "desired_pose", 10)
        self.create_subscription(PoseStamped, "estimated_pose", self._estimated_pose_cb, 10)
        self.create_subscription(JointState, "cable_joint_commands", self._cable_commands_cb, 10)

        self._values = {
            "cable_1_joint": 0.0,
            "cable_2_joint": 0.0,
            "cable_3_joint": 0.0,
        }
        self._desired_xyz   = [0.0, 0.0, 0.0]
        self._estimated_xyz = [0.0, 0.0, 0.0]
        self._scales = {}
        self._slider_min = 0.0
        self._slider_max = 0.15

        slider_min = float(self.get_parameter("slider_min").value)
        slider_max = float(self.get_parameter("slider_max").value)
        slider_resolution = float(self.get_parameter("slider_resolution").value)
        self._slider_min  = slider_min
        self._slider_max  = slider_max

        self._root = tk.Tk()
        self._root.title("Cable Robot Control")
        self._root.geometry("360x560")
        self._root.protocol("WM_DELETE_WINDOW", self._handle_close)

        # --- Sliders ---
        tk.Label(self._root, text="Cable Tension Control", font=("Arial", 14, "bold")).pack(pady=10)

        for joint_name in self._values:
            frame = tk.Frame(self._root)
            frame.pack(fill="x", padx=14, pady=8)
            tk.Label(frame, text=joint_name, anchor="w").pack(fill="x")
            scale = tk.Scale(
                frame,
                from_=slider_min, to=slider_max, resolution=slider_resolution,
                orient=tk.HORIZONTAL, length=300,
                command=lambda value, joint=joint_name: self._update_value(joint, value),
            )
            scale.set(0.0)
            scale.pack()
            self._scales[joint_name] = scale

        tk.Label(self._root, text="0 = reposo / suelto    MAX = tensión máxima", font=("Arial", 9)).pack(pady=2)
        tk.Button(self._root, text="Home (soltar todos)", command=self._center_sliders).pack(pady=8)

        # --- Separador ---
        tk.Frame(self._root, height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, padx=10, pady=6)

        # --- Sección IK ---
        tk.Label(self._root, text="Posición objetivo (IK)", font=("Arial", 11, "bold")).pack()

        xyz_frame = tk.Frame(self._root)
        xyz_frame.pack(padx=14, pady=6)
        self._xyz_entries = []
        for i, axis in enumerate(["X (m)", "Y (m)", "Z (m)"]):
            tk.Label(xyz_frame, text=axis, width=6, anchor="e").grid(row=0, column=i*2, padx=(8, 2))
            entry = tk.Entry(xyz_frame, width=8)
            entry.insert(0, "0.0")
            entry.grid(row=0, column=i*2+1, padx=(0, 8))
            self._xyz_entries.append(entry)

        tk.Button(self._root, text="Ir a posición", command=self._send_pose).pack(pady=4)

        self._estimated_label = tk.Label(self._root, text="Estimado: x=—  y=—  z=—", font=("Arial", 9))
        self._estimated_label.pack()
        self._error_label = tk.Label(self._root, text="Error: — mm", font=("Arial", 9, "bold"))
        self._error_label.pack()

        self.create_timer(0.05, self._spin_gui)
        self.create_timer(0.1, self._publish_joint_state)

    def _update_value(self, joint_name: str, value: str) -> None:
        self._values[joint_name] = float(value)

    def _center_sliders(self) -> None:
        for child in self._root.winfo_children():
            if isinstance(child, tk.Frame):
                for widget in child.winfo_children():
                    if isinstance(widget, tk.Scale):
                        widget.set(0.0)
        for name in self._values:
            self._values[name] = -0.005
        self._root.after(500, self._finish_home)

    def _finish_home(self) -> None:
        for name in self._values:
            self._values[name] = 0.0

    def _send_pose(self) -> None:
        try:
            x = float(self._xyz_entries[0].get())
            y = float(self._xyz_entries[1].get())
            z = float(self._xyz_entries[2].get())
        except ValueError:
            self.get_logger().warn("Valores x,y,z no válidos")
            return
        self._desired_xyz = [x, y, z]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._pose_publisher.publish(msg)
        self._refresh_error()

    def _cable_commands_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._scales:
                clamped = max(self._slider_min, min(self._slider_max, pos))
                self._values[name] = clamped
                self._scales[name].set(clamped)

    def _estimated_pose_cb(self, msg: PoseStamped) -> None:
        self._estimated_xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self._refresh_error()

    def _refresh_error(self) -> None:
        ex, ey, ez = self._estimated_xyz
        self._estimated_label.config(text=f"Estimado: x={ex:.3f}  y={ey:.3f}  z={ez:.3f}")
        dx = self._desired_xyz[0] - ex
        dy = self._desired_xyz[1] - ey
        dz = self._desired_xyz[2] - ez
        err_mm = (dx**2 + dy**2 + dz**2) ** 0.5 * 1000
        self._error_label.config(text=f"Error: {err_mm:.1f} mm")

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._values.keys())
        msg.position = [self._values[name] for name in msg.name]
        self._publisher.publish(msg)

    def _spin_gui(self) -> None:
        try:
            self._root.update_idletasks()
            self._root.update()
        except tk.TclError:
            pass

    def _handle_close(self) -> None:
        self.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = CableSliderGui()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
