#!/usr/bin/env python3
"""ROS 2 Node: Task Orchestrator — The Brain.

Implements a Finite State Machine (FSM) that governs the entire robotic cycle.

Uses **direct geometric IK** (no MoveIt service required) for reliable, fast
motion commands that are guaranteed to work with the joint controller.

Subscribes:  /vision/target_point  (geometry_msgs/Point)
Published:   /arm_controller/joint_trajectory
             /gripper_controller/joint_trajectory
             /robot_state  (std_msgs/String)
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── ROBOT GEOMETRY (from URDF) ───────────────────────────────────────────────
# Upper arm (shoulder→elbow pivot) and forearm (elbow→wrist pivot) lengths
# Measured from URDF origin offsets along Z-axis
L1 = 0.1176   # shoulder to elbow (m)  – from elbow_joint origin z=0.117647
L2 = 0.095    # elbow to wrist    (m)  – estimated from wrist_roll origin


def geometric_ik(x: float, y: float, z: float):
    """2-link planar arm geometric IK in the arm's sagittal plane.

    Args:
        x, y: horizontal target (metres) in robot base frame
        z:    vertical height  (metres) above base

    Returns:
        (waist, shoulder, elbow, wrist_roll, wrist_pitch) in radians
        or None if unreachable.
    """
    # Waist: rotate base to face the target horizontally
    waist = math.atan2(y, x)
    waist = max(-1.5708, min(1.5708, waist))

    # Horizontal reach (radial distance in XY plane)
    r = math.sqrt(x * x + y * y)
    # Height offset relative to shoulder (approx shoulder at z≈0.02m above base)
    delta_z = z - 0.02

    dist = math.sqrt(r * r + delta_z * delta_z)

    # Reachability check
    if dist > L1 + L2 - 0.005:
        return None

    # Law of cosines for elbow angle
    cos_elbow = (L1 * L1 + L2 * L2 - dist * dist) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.pi - math.acos(cos_elbow)           # bend forward
    elbow = max(-1.2217, min(1.2217, elbow))

    # Shoulder angle: angle to target + correction for elbow
    alpha = math.atan2(delta_z, r)
    cos_shoulder = (L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist)
    cos_shoulder = max(-1.0, min(1.0, cos_shoulder))
    beta = math.acos(cos_shoulder)
    shoulder = alpha + beta                           # lift up
    shoulder = max(-1.0472, min(1.0472, shoulder))

    # Wrist keeps gripper horizontal / pointing down
    wrist_pitch = -(shoulder + elbow - math.pi / 2)
    wrist_pitch = max(-1.5708, min(1.5708, wrist_pitch))

    wrist_roll = 0.0

    return (waist, shoulder, elbow, wrist_roll, wrist_pitch)


class TaskOrchestratorNode(Node):
    """Pick-and-Place FSM using Geometric IK — no MoveIt service needed."""

    # ── State constants ───────────────────────────────────────────────────────
    S_IDLE             = "IDLE"
    S_HOME_SCANNING    = "HOME / SCANNING"
    S_APPROACHING      = "APPROACHING"
    S_SETTLING         = "SETTLING"
    S_DESCENDING       = "DESCENDING"
    S_GRASPING         = "GRASPING"
    S_LIFTING          = "LIFTING"
    S_RETURN_HOME_FULL = "RETURN HOME"
    S_DELIVERING       = "DELIVERING"
    S_RELEASING        = "RELEASING"
    S_RETURN_HOME_DONE = "RESETTING"

    ARM_JOINTS = [
        "waist_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_roll_joint",
        "wrist_pitch_joint",
    ]

    def __init__(self) -> None:
        super().__init__("task_orchestrator_node")

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._vision_sub = self.create_subscription(
            Point, "/vision/target_point", self._on_vision, 10)

        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self._gripper_traj_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        self._state_pub = self.create_publisher(String, "/robot_state", 10)

        # FSM timer — 10 Hz
        self.create_timer(0.1, self._fsm_tick)

        # ── Configurable positions ────────────────────────────────────────
        self.declare_parameter("hover_z", 0.08)    # safe transit height
        self.declare_parameter("drop_x",  0.10)    # drop zone X
        self.declare_parameter("drop_y", -0.10)    # drop zone Y

        # ── Internal state ────────────────────────────────────────────────
        self._state         = self.S_IDLE
        self._state_timer   = time.monotonic()
        self._action_done   = False
        self._target_x      = 0.0
        self._target_y      = 0.0
        self._target_locked = False

        self.get_logger().info(
            "🧠 Task Orchestrator (Geometric IK Edition) alive. "
            "Waiting for a locked target on /vision/target_point …"
        )

    # ── Vision subscriber ─────────────────────────────────────────────────────

    def _on_vision(self, msg: Point) -> None:
        """Accept coordinates only while idle or scanning."""
        if self._state in (self.S_IDLE, self.S_HOME_SCANNING):
            self._target_x      = msg.x
            self._target_y      = msg.y
            self._target_locked = True
            self.get_logger().info(
                f"🎯  Target locked → x={msg.x:.3f} y={msg.y:.3f}"
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            self.get_logger().info(f"🦾  STATE ➜  {new_state}")
            self._state_pub.publish(String(data=new_state))
        self._state       = new_state
        self._state_timer = time.monotonic()
        self._action_done = False

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_timer

    # ── Motion commands ───────────────────────────────────────────────────────

    def _send_joints(self, positions: list, duration: float) -> None:
        """Publish a JointTrajectory with the given joint positions."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = self.ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec    = int(duration)
        nsec   = int((duration - sec) * 1_000_000_000)
        pt.time_from_start.sec     = sec
        pt.time_from_start.nanosec = nsec
        msg.points.append(pt)
        self._arm_traj_pub.publish(msg)
        self.get_logger().debug(
            f"   → joints {[f'{p:.3f}' for p in positions]}"
        )

    def _send_arm_home(self, duration: float = 1.5) -> None:
        """Return all joints to zero (upright/home position)."""
        self._send_joints([0.0, 0.0, 0.0, 0.0, 0.0], duration)
        self.get_logger().info("🏠  Arm → HOME")

    def _send_arm_to(self, x: float, y: float, z: float,
                     duration: float = 1.5) -> bool:
        """Compute geometric IK and send trajectory. Returns True on success."""
        result = geometric_ik(x, y, z)
        if result is None:
            self.get_logger().warn(
                f"⚠️  Geometric IK failed for ({x:.3f}, {y:.3f}, {z:.3f}) — "
                "target unreachable."
            )
            return False
        waist, shoulder, elbow, wrist_roll, wrist_pitch = result
        self.get_logger().info(
            f"📐  IK → waist={math.degrees(waist):.1f}° "
            f"shoulder={math.degrees(shoulder):.1f}° "
            f"elbow={math.degrees(elbow):.1f}°"
        )
        self._send_joints([waist, shoulder, elbow, wrist_roll, wrist_pitch], duration)
        return True

    def _set_gripper(self, value: float, duration: float = 0.5) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = ["gripper_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(value)]
        sec  = int(duration)
        nsec = int((duration - sec) * 1_000_000_000)
        pt.time_from_start.sec     = sec
        pt.time_from_start.nanosec = nsec
        msg.points.append(pt)
        self._gripper_traj_pub.publish(msg)

    # ── FSM ───────────────────────────────────────────────────────────────────

    def _fsm_tick(self) -> None:
        """One FSM tick at 10 Hz. Each state fires its action ONCE."""
        hover_z = self.get_parameter("hover_z").value
        drop_x  = self.get_parameter("drop_x").value
        drop_y  = self.get_parameter("drop_y").value

        # Fires True only the first time we enter each state
        first = not self._action_done

        # ── 1. IDLE ───────────────────────────────────────────────────────
        if self._state == self.S_IDLE:
            if self._target_locked:
                self._set_state(self.S_HOME_SCANNING)

        # ── 2. HOME / SCANNING ───────────────────────────────────────────
        elif self._state == self.S_HOME_SCANNING:
            if first:
                self._send_arm_home()
                self._set_gripper(0.0)
                self._action_done = True
            if self._target_locked and self._elapsed() > 1.5:
                self._set_state(self.S_APPROACHING)

        # ── 3. APPROACHING (hover above target) ──────────────────────────
        elif self._state == self.S_APPROACHING:
            if first:
                ok = self._send_arm_to(self._target_x, self._target_y,
                                       hover_z, duration=1.5)
                self._action_done = True
                if not ok:
                    # IK failed → abort back to scanning
                    self._target_locked = False
                    self._set_state(self.S_HOME_SCANNING)
                    return
            if self._elapsed() > 2.0:
                self._set_state(self.S_SETTLING)

        # ── 4. SETTLING ──────────────────────────────────────────────────
        elif self._state == self.S_SETTLING:
            if self._elapsed() > 0.6:
                self._set_state(self.S_DESCENDING)

        # ── 5. DESCENDING (to table level) ───────────────────────────────
        elif self._state == self.S_DESCENDING:
            if first:
                self._send_arm_to(self._target_x, self._target_y,
                                  0.02, duration=1.0)
                self._action_done = True
            if self._elapsed() > 1.5:
                self._set_state(self.S_GRASPING)

        # ── 6. GRASPING ──────────────────────────────────────────────────
        elif self._state == self.S_GRASPING:
            if first:
                self._set_gripper(0.5)
                self._action_done = True
            if self._elapsed() > 1.0:
                self._set_state(self.S_LIFTING)

        # ── 7. LIFTING ───────────────────────────────────────────────────
        elif self._state == self.S_LIFTING:
            if first:
                self._send_arm_to(self._target_x, self._target_y,
                                  hover_z, duration=1.0)
                self._action_done = True
            if self._elapsed() > 1.5:
                self._set_state(self.S_RETURN_HOME_FULL)

        # ── 8. RETURN HOME (before delivery) ─────────────────────────────
        elif self._state == self.S_RETURN_HOME_FULL:
            if first:
                self._send_arm_home()
                self._action_done = True
            if self._elapsed() > 2.0:
                self._set_state(self.S_DELIVERING)

        # ── 9. DELIVERING (over drop zone) ───────────────────────────────
        elif self._state == self.S_DELIVERING:
            if first:
                self._send_arm_to(drop_x, drop_y, hover_z, duration=1.5)
                self._action_done = True
            if self._elapsed() > 2.0:
                self._set_state(self.S_RELEASING)

        # ── 10. RELEASING ────────────────────────────────────────────────
        elif self._state == self.S_RELEASING:
            if first:
                self._set_gripper(0.0)
                self._action_done = True
            if self._elapsed() > 1.0:
                self._set_state(self.S_RETURN_HOME_DONE)

        # ── 11. RESETTING ─────────────────────────────────────────────────
        elif self._state == self.S_RETURN_HOME_DONE:
            if first:
                self._send_arm_home()
                self._action_done = True
            if self._elapsed() > 2.0:
                self._target_locked = False
                self._set_state(self.S_HOME_SCANNING)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskOrchestratorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
