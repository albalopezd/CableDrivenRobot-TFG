import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CableSliderGui(Node):
    def __init__(self) -> None:
        super().__init__("cable_slider_gui")
        self.declare_parameter("slider_min", -0.04)
        self.declare_parameter("slider_max", 0.04)

        self._publisher = self.create_publisher(JointState, "manual_cable_joint_commands", 10)
        self._values = {
            "cable_1_joint": 0.0,
            "cable_2_joint": 0.0,
            "cable_3_joint": 0.0,
        }

        slider_min = float(self.get_parameter("slider_min").value)
        slider_max = float(self.get_parameter("slider_max").value)

        self._root = tk.Tk()
        self._root.title("Cable Length Control")
        self._root.geometry("340x340")
        self._root.protocol("WM_DELETE_WINDOW", self._handle_close)

        title = tk.Label(self._root, text="Cable Length Delta", font=("Arial", 14, "bold"))
        title.pack(pady=10)

        for joint_name in self._values:
            frame = tk.Frame(self._root)
            frame.pack(fill="x", padx=14, pady=8)

            label = tk.Label(frame, text=joint_name, anchor="w")
            label.pack(fill="x")

            scale = tk.Scale(
                frame,
                from_=slider_min,
                to=slider_max,
                resolution=0.001,
                orient=tk.HORIZONTAL,
                length=260,
                command=lambda value, joint=joint_name: self._update_value(joint, value),
            )
            scale.set(0.0)
            scale.pack()

        info = tk.Label(
            self._root,
            text="Positive = longer cable, negative = shorter cable",
            font=("Arial", 9),
        )
        info.pack(pady=2)

        center_button = tk.Button(self._root, text="Center", command=self._center_sliders)
        center_button.pack(pady=8)

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
