#!/usr/bin/env python3
"""ROS 2 GUI Teleoperation Node.

Publishes sensor_msgs/JointState from Tkinter sliders at 20 Hz.
This node is the bridge between the human operator and the ROS 2 system.

Topics Published:
    /joint_states  (sensor_msgs/JointState)
"""

from __future__ import annotations

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk


# Joint configuration: (Display Name, ROS Joint Name, Min Radians, Max Radians)
JOINTS_INFO: list[tuple[str, str, float, float]] = [
    ("Waist Base",  "waist_joint",       -1.57, 1.57),
    ("Shoulder",    "shoulder_joint",    -1.04, 1.04), # 30 to 150 deg
    ("Elbow",       "elbow_joint",       -1.22, 1.22), # 20 to 160 deg
    ("Wrist Roll",  "wrist_roll_joint",  -1.57, 1.57),
    ("Wrist Pitch", "wrist_pitch_joint", -1.57, 1.57),
    ("Gripper",     "gripper_joint",     -0.5,  0.5), # 20 to 160 deg mapping
]

# All joints including mimic links (must match URDF joint names exactly)
ALL_JOINT_NAMES: list[str] = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_roll_joint", "wrist_pitch_joint", "gripper_joint",
    "gripper_right_joint", "gear_left_joint", "gear_right_joint",
    "revolute_7_0", "revolute_8_0",
]


class JointPublisherNode(Node):
    """Broadcasts current joint positions at a fixed rate for RViz and hardware."""

    def __init__(self) -> None:
        super().__init__("gui_teleop_node")
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(0.05, self._publish)  # 20 Hz
        self.joint_values: dict[str, float] = {name: 0.0 for name in ALL_JOINT_NAMES}

    def _publish(self) -> None:
        """Compute mimic joints and broadcast the full JointState message."""
        gv = self.joint_values["gripper_joint"]
        # Gripper linkage mimicry (matches URDF mimic tags)
        self.joint_values["gripper_right_joint"] = -gv
        self.joint_values["gear_left_joint"]      =  gv
        self.joint_values["gear_right_joint"]     = -gv
        self.joint_values["revolute_7_0"]         = -gv
        self.joint_values["revolute_8_0"]         =  gv

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINT_NAMES
        msg.position = [self.joint_values[n] for n in ALL_JOINT_NAMES]
        self._pub.publish(msg)


class RobotTeleopUI(tk.Tk):
    """Minimal dark-theme Tkinter window with per-joint sliders."""

    def __init__(self, ros_node: JointPublisherNode) -> None:
        super().__init__()
        self._node = ros_node
        self.title("Robotic Arm — Manual Teleoperation")
        self.geometry("600x480")
        self.configure(bg="#1E1E1E")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build_ui()

    def _build_ui(self) -> None:
        tk.Label(
            self, text="Manual Teleoperation",
            font=("Arial", 18, "bold"), bg="#1E1E1E", fg="#FFFFFF"
        ).pack(pady=12)
        tk.Label(
            self, text="Publishes /joint_states at 20 Hz → RViz2 + Arduino Serial Bridge",
            font=("Arial", 9), bg="#1E1E1E", fg="#888888"
        ).pack(pady=(0, 8))

        panel = tk.Frame(self, bg="#2D2D2D", bd=1, relief=tk.RIDGE)
        panel.pack(fill=tk.BOTH, expand=True, padx=20, pady=8)

        tk.Label(
            panel, text="Joint Sliders", font=("Arial", 13, "bold"),
            bg="#2D2D2D", fg="#4CAF50"
        ).pack(pady=8)

        self._sliders: dict[str, tk.Scale] = {}
        self._val_labels: dict[str, tk.Label] = {}

        for display_name, joint_name, min_rad, max_rad in JOINTS_INFO:
            row = tk.Frame(panel, bg="#2D2D2D")
            row.pack(fill=tk.X, padx=20, pady=4)

            tk.Label(
                row, text=display_name, width=14, anchor="w",
                bg="#2D2D2D", fg="#CCCCCC", font=("Arial", 11)
            ).pack(side=tk.LEFT)

            sl = tk.Scale(
                row, from_=min_rad, to=max_rad, resolution=0.01,
                orient=tk.HORIZONTAL, bg="#2D2D2D", fg="white",
                highlightthickness=0, length=290,
                command=lambda v, n=joint_name: self._on_slider(n, v),
            )
            sl.pack(side=tk.LEFT, fill=tk.X, expand=True)

            lbl = tk.Label(
                row, text="0.00 rad", width=9,
                bg="#2D2D2D", fg="#9E9E9E", font=("Arial", 9)
            )
            lbl.pack(side=tk.LEFT, padx=(6, 0))

            self._sliders[joint_name] = sl
            self._val_labels[joint_name] = lbl

    def _on_slider(self, joint_name: str, value: str) -> None:
        rad = float(value)
        self._node.joint_values[joint_name] = rad
        self._val_labels[joint_name].config(text=f"{rad:+.2f} rad")

    def _on_close(self) -> None:
        """Gracefully shut down ROS 2 before destroying the window."""
        if rclpy.ok():
            rclpy.shutdown()
        self.destroy()


def main() -> None:
    rclpy.init()
    ros_node = JointPublisherNode()

    # Spin ROS 2 in background so Tkinter owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = RobotTeleopUI(ros_node)
    app.mainloop()

    ros_node.destroy_node()


if __name__ == "__main__":
    main()
