#!/usr/bin/env python3
"""ROS 2 Serial Bridge Node.

Subscribes to /joint_states and forwards angle commands to the Arduino
via USB serial using the S-Protocol ASCII format.

Topics Subscribed:
    /joint_states          (sensor_msgs/JointState)

Topics Published:
    /hardware/serial_feedback  (std_msgs/String)   — Raw Arduino responses
    /hardware/ack_alive        (std_msgs/Bool)      — Heartbeat health flag
"""

from __future__ import annotations

import math
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class SerialBridgeNode(Node):
    """Converts ROS 2 joint states to Arduino S-Protocol commands."""

    # Home angles in degrees — applied on startup and reconnect
    HOME_ANGLES: dict[str, int] = {
        "waist_joint":       90,
        "shoulder_joint":    90,
        "elbow_joint":       90,
        "wrist_roll_joint":  90,
        "wrist_pitch_joint": 90,
        "gripper_joint":     30,
    }

    # Hardware Limits (Degrees) for internal safety clamping
    HARDWARE_LIMITS: dict[str, tuple[int, int]] = {
        "waist_joint":       (0,  180),
        "shoulder_joint":    (30, 150),
        "elbow_joint":       (20, 160),
        "wrist_roll_joint":  (0,  180),
        "wrist_pitch_joint": (0,  180),
        "gripper_joint":     (20, 160),
    }

    # Sequence for serial transmission (S-Protocol)
    JOINT_ORDER: list[str] = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_roll_joint", "wrist_pitch_joint", "gripper_joint"
    ]

    def __init__(self) -> None:
        super().__init__("serial_bridge_node")

        # Parameters (configurable from launch file)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate",   115200)
        self.declare_parameter("rate_hz",     20)

        port    = self.get_parameter("serial_port").value
        baud    = self.get_parameter("baud_rate").value
        rate_hz = self.get_parameter("rate_hz").value

        # Publishers
        self._feedback_pub  = self.create_publisher(String, "/hardware/serial_feedback", 10)
        self._ack_alive_pub = self.create_publisher(Bool,   "/hardware/ack_alive",       10)

        # Subscriber
        self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)

        # Timers
        self.create_timer(1.0 / rate_hz, self._timer_transmit)   # Transmit at fixed rate
        self.create_timer(0.01,          self._read_feedback)     # Poll Arduino responses
        self.create_timer(0.2,           self._publish_health)    # ACK health broadcast

        # Internal state
        self._angles: dict[str, int]     = dict(self.HOME_ANGLES)
        self._angles_changed: bool        = True   # Force send on startup
        self._active: bool                = False
        self._ser: serial.Serial | None   = None
        self._debug_counter: int          = 0
        self._last_ack_time: float        = time.monotonic()
        self._ack_timeout_sec: float      = 2.0

        self._open_port(port, baud)
        self.get_logger().info("Serial Bridge Node ready.")

    # ── Port Management ───────────────────────────────────────────────────────

    def _open_port(self, port: str, baud: int) -> None:
        """Open the serial port and wait for Arduino to boot."""
        try:
            self._ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Port opened. Waiting 2s for Arduino to boot...")
            time.sleep(2)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._active = True
            self.get_logger().info(f"✅ Connected to Arduino on {port} @ {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(
                f"❌ Cannot open {port}: {e}\n"
                f"   → Check: ls /dev/ttyACM* /dev/ttyUSB*"
            )

    def _try_reconnect(self) -> None:
        """Attempt to reopen the serial port after a disconnect."""
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.get_logger().info(f"Reconnecting to {port}...")
        try:
            self._ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._active = True
            self.get_logger().info("✅ Reconnected successfully.")
        except Exception as e:
            self.get_logger().debug(f"Reconnect failed: {e}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        """Convert incoming radians to degrees and buffer for next transmit tick."""
        if not self._active:
            return

        for name, rad in zip(msg.name, msg.position):
            if name not in self._angles:
                continue
            
            # 1. Conversion
            if name == "gripper_joint":
                # Custom linear mapping: -0.5 rad → 20°, +0.5 rad → 160°
                deg = 140.0 * rad + 90.0
            else:
                # Standard: 0 rad → 90°
                deg = math.degrees(rad) + 90.0

            # 2. Safety Clamp matching hardware limits
            min_deg, max_deg = self.HARDWARE_LIMITS.get(name, (0, 180))
            self._angles[name] = int(round(max(min_deg, min(max_deg, deg))))

        self._angles_changed = True

    # ── Timers ────────────────────────────────────────────────────────────────

    def _timer_transmit(self) -> None:
        """Send buffered angles to Arduino at the configured rate."""
        if not self._active:
            self._debug_counter += 1
            if self._debug_counter % 100 == 0:
                self._try_reconnect()
            return

        if not self._angles_changed:
            return

        values = [str(self._angles[j]) for j in self.JOINT_ORDER]
        cmd = f"S:{','.join(values)}\n"

        try:
            self._ser.write(cmd.encode("utf-8"))
            self._angles_changed = False
            self._debug_counter += 1
            # Log once per second (every 20 ticks at 20 Hz)
            if self._debug_counter % 20 == 0:
                self.get_logger().info(f"TX → {cmd.strip()}")
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"❌ Serial write failed: {e}")
            self._active = False
            try:
                self._ser.close()
            except Exception:
                pass

    def _read_feedback(self) -> None:
        """Non-blocking read of Arduino responses and publish to /hardware/serial_feedback."""
        if not self._active or not self._ser:
            return
        try:
            while self._ser.in_waiting > 0:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    if line.startswith("ACK:"):
                        self._last_ack_time = time.monotonic()
                    else:
                        self.get_logger().info(f"Arduino: {line}")
                    self._feedback_pub.publish(String(data=line))
        except Exception:
            pass  # Transient read errors are non-fatal

    def _publish_health(self) -> None:
        """Publish whether we received an ACK within the timeout window."""
        elapsed = time.monotonic() - self._last_ack_time
        self._ack_alive_pub.publish(Bool(data=(self._active and elapsed <= self._ack_timeout_sec)))


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._active and node._ser:
            node._ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
