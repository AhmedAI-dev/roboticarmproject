#!/usr/bin/env python3
"""ROS 2 Node: High-Precision Vision Perception.

Detects small cubic objects (like wooden/plastic toy blocks) using
HSV color thresholds with a strict geometric shape filter.
Large objects (clothing, hands, tables) are explicitly rejected.

Subscribes:  /robot_state   (std_msgs/String)  — Brain status for HUD overlay
Published:   /vision/target_point (geometry_msgs/Point) — Verified target coords
             /vision/image_raw    (sensor_msgs/Image)   — Annotated camera feed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class VisionPerceptionNode(Node):
    """Precision block detector with confidence gating and HUD overlay."""

    # ── HSV Color Bounds ──────────────────────────────────────────────────────
    # Tuned for small solid-color blocks under indoor LED lighting.
    COLOR_BOUNDS: dict = {
        "RED":    [(np.array([0,  150, 80]),  np.array([10, 255, 255])),
                   (np.array([168, 150, 80]), np.array([180, 255, 255]))],
        "BLUE":   [(np.array([100, 150, 60]), np.array([130, 255, 255]))],
        "YELLOW": [(np.array([22, 150, 80]),  np.array([35, 255, 255]))],
        "GREEN":  [(np.array([40, 120, 60]),  np.array([80, 255, 255]))],
    }

    # ── Block Size Constraints (pixels²) ─────────────────────────────────────
    # Calibrated for a FORWARD-FACING camera at 10 cm height.
    # Blocks 3-6 cm at 8-20 cm working distance will appear 800-8000 px².
    MIN_AREA =  600    # px²  — smaller noise / fingertip fragments rejected
    MAX_AREA = 12000   # px²  — large background objects rejected

    # ── Shape Constraints ─────────────────────────────────────────────────────
    MIN_ASPECT  = 0.50   # more lenient — side view of blocks is rectangular
    MAX_ASPECT  = 2.00
    MIN_SOLIDITY = 0.78  # slightly relaxed for angled/perspective view

    # ── Confidence Parameters ─────────────────────────────────────────────────
    MAX_CONFIDENCE = 30     # ~1.5 s at 20 Hz (faster lock-on)
    DECAY_RATE     = 4      # faster decay when target lost

    def __init__(self) -> None:
        super().__init__("vision_perception_node")

        # ── ROS Parameters ────────────────────────────────────────────────
        self.declare_parameter("camera_id",       0)
        self.declare_parameter("target_color",    "RED")
        self.declare_parameter("debug_window",    True)

        # ── Camera Geometry ─────────────────────────────────────────────────
        # Camera sits rigidly next to the WAIST cylinder of the arm.
        # It looks HORIZONTALLY FORWARD toward the gripper's working area.
        # Physical placement:
        #   • Glued/clamped right beside the waist servo (≈2 cm lateral offset)
        #   • 10 cm above the table surface
        #   • Zero tilt — perfectly level with the horizon
        # ─────────────────────────────────────────────────────────────────────
        # cam_x : how far FORWARD the camera is from the base centre (m)
        # cam_y : lateral offset from the base centre (+Y = left of arm) (m)
        # cam_height_over_table : camera height above the table plane (m)
        # cam_fov_h / cam_fov_v : camera horizontal/vertical FOV in degrees
        self.declare_parameter("cam_x",                0.02)  # 2 cm in front of waist
        self.declare_parameter("cam_y",                0.03)  # 3 cm to the side of waist
        self.declare_parameter("cam_z",                0.10)  # 10 cm above table
        self.declare_parameter("cam_height_over_table",0.10)  # same — height assumption
        self.declare_parameter("cam_fov_h",            70.0)  # typical USB webcam H-FOV
        self.declare_parameter("cam_fov_v",            50.0)  # typical USB webcam V-FOV
        self.declare_parameter("table_z",              0.00)  # table plane in robot frame

        camera_id = self.get_parameter("camera_id").value

        # ── Camera Auto-Detection ─────────────────────────────────────────
        self._camera_active = False
        self.cap = None
        
        # We try the user-provided camera_id first, then fallback to testing 0 through 4
        for test_id in [camera_id] + list(range(5)):
            cap = cv2.VideoCapture(test_id)
            if cap.isOpened():
                self.cap = cap
                self._camera_active = True
                camera_id = test_id
                self.get_logger().info(f"🎥 Camera successfully connected on index {test_id}!")
                break
            cap.release()

        if not self._camera_active:
            self.get_logger().error("❌ Cannot open ANY camera (tried 0-4). Running blind.")
        else:
            # Boost resolution for better detection accuracy
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS,       1)  # enable autofocus if supported
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,   1)  # auto exposure on
            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(
                f"📐 Camera resolution: {int(actual_w)}×{int(actual_h)}")

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._target_pub = self.create_publisher(Point,  "/vision/target_point", 10)
        self._image_pub  = self.create_publisher(Image,  "/vision/image_raw",    10)
        self._state_sub  = self.create_subscription(
            String, "/robot_state", self._on_robot_state, 10)

        self._bridge = CvBridge()
        self.create_timer(0.05, self._process)   # 20 Hz

        # ── State ─────────────────────────────────────────────────────────
        self._confidence      = 0
        self._robot_state_str = "IDLE"           # latest text from Brain
        self._last_target     = (0.0, 0.0)       # last confirmed real-world pos

        color = self.get_parameter("target_color").value
        self.get_logger().info(
            f"👁️ Vision Perception ready. "
            f"Camera {camera_id} | Target color: {color}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_robot_state(self, msg: String) -> None:
        """Receive current FSM state text from the Brain for HUD display."""
        self._robot_state_str = msg.data

    # ── Internal Helpers ──────────────────────────────────────────────────────

    def _build_mask(self, hsv: np.ndarray) -> np.ndarray | None:
        """Return a binary mask for the configured target color."""
        color = self.get_parameter("target_color").value.upper()
        bounds = self.COLOR_BOUNDS.get(color)
        if bounds is None:
            self.get_logger().error(f"Unknown target_color '{color}'")
            return None

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in bounds:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Morphological clean-up: remove noise, fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _is_valid_block(self, contour: np.ndarray) -> bool:
        """Return True only if the contour passes ALL shape filters."""
        area = cv2.contourArea(contour)

        # 1. Area gate — reject everything outside the expected block size
        if not (self.MIN_AREA < area < self.MAX_AREA):
            return False

        # 2. Aspect ratio — blocks are roughly square
        _, _, w, h = cv2.boundingRect(contour)
        if h == 0:
            return False
        ratio = float(w) / h
        if not (self.MIN_ASPECT <= ratio <= self.MAX_ASPECT):
            return False

        # 3. Solidity — blocks are convex; clothing folds are concave
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area < 1:
            return False
        solidity = area / hull_area
        if solidity < self.MIN_SOLIDITY:
            return False

        return True

    def _px_to_meters(self, px: int, py: int,
                       width: int, height: int) -> tuple[float, float]:
        """Project a pixel (px, py) to robot-base XY coordinates.

        Camera model: fixed, rigidly mounted very close to the arm base,
        looking HORIZONTALLY FORWARD along the arm's default reach direction
        (+X axis of the robot frame).  The camera does NOT tilt up or down.

        Derivation (pin-hole, no distortion):
          The camera's optical axis is horizontal at height cam_z.
          An object sitting ON the table (height = table_z) is at a depth:
              depth = cam_z / tan(v_angle)     [horizontal = v_angle=0 → ∞]
          where v_angle is the downward tilt implied by the pixel row:
              v_angle = ((cy - py) / cy) * (fov_v / 2)   [positive = looking down]

          Lateral offset (left–right):  uses horizontal FOV similarly.
        """
        import math

        cam_x    = self.get_parameter("cam_x").value
        cam_y    = self.get_parameter("cam_y").value
        cam_z    = self.get_parameter("cam_height_over_table").value
        fov_h    = math.radians(self.get_parameter("cam_fov_h").value)
        fov_v    = math.radians(self.get_parameter("cam_fov_v").value)

        cx = width  / 2.0
        cy = height / 2.0

        # Normalised pixel offsets (-1 … 1)
        norm_x = (px - cx) / cx   # left → negative
        norm_y = (cy - py) / cy   # up   → positive (looking up)

        # Angular offsets from optical axis
        angle_lat  = norm_x * (fov_h / 2)   # yaw offset  (left-right)
        angle_vert = norm_y * (fov_v / 2)   # pitch offset (positive = above horizon)

        # Objects below the horizon give a downward angle; invert for depth calc
        tilt_down = -angle_vert   # positive = camera pitched down toward object

        if tilt_down <= 0.001:    # object on or above horizon → unreachable / infinite
            return cam_x, cam_y

        # Geometric depth from camera to object on table plane
        depth = cam_z / math.tan(tilt_down)   # metres along +X

        # Robot-frame position
        robot_x = cam_x + depth                      # forward from base
        robot_y = cam_y + depth * math.tan(angle_lat) # lateral from base

        return robot_x, robot_y

    # ── HUD Overlay Helpers ────────────────────────────────────────────────────

    def _draw_hud(self, frame: np.ndarray,
                  confidence: int, target_found: bool,
                  rx: float = 0.0, ry: float = 0.0) -> None:
        """Draw a fixed-position HUD panel in the top-left corner."""
        h, w = frame.shape[:2]

        # Semi-transparent background for HUD panel
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (320, 95), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        color   = self.get_parameter("target_color").value
        bar_max = 300
        bar_w   = int((confidence / self.MAX_CONFIDENCE) * bar_max)
        bar_clr = (0, 255, 0) if target_found else (0, 220, 255)

        # Row 1: Brain state
        cv2.putText(frame, f"STATE: {self._robot_state_str}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # Row 2: Active target color
        cv2.putText(frame, f"TARGET COLOR: {color}",
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 220, 255), 1)

        # Row 3: Confidence bar + label
        cv2.rectangle(frame, (8, 52), (8 + bar_max, 66), (60, 60, 60), -1)
        if bar_w > 0:
            cv2.rectangle(frame, (8, 52), (8 + bar_w, 66), bar_clr, -1)
        status_lbl = "LOCKED ✔" if target_found else f"{int(confidence / self.MAX_CONFIDENCE * 100)}%"
        cv2.putText(frame, status_lbl, (8 + bar_max + 6, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, bar_clr, 1)

        # Row 4: Robot coordinates when locked
        if target_found:
            cv2.putText(frame, f"X:{rx:.3f}m  Y:{ry:.3f}m",
                        (8, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (100, 255, 100), 1)

    def _draw_target_annotation(self, frame: np.ndarray,
                                 contour: np.ndarray,
                                 cx: int, cy: int,
                                 confidence: int, locked: bool,
                                 over_limit: bool = False) -> None:
        """Draw bounding box and crosshair on the detected block."""
        x, y, w, h = cv2.boundingRect(contour)
        
        if over_limit:
            box_clr = (0, 0, 255) # RED
            label = "OUT OF REACH"
        elif locked:
            box_clr = (0, 255, 0) # GREEN
            label = "LOCKED"
        else:
            box_clr = (0, 220, 255) # YELLOW
            label = "SCANNING..."

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_clr, 2)
        # Cross-hair at centroid
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), box_clr, 1)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), box_clr, 1)

        cv2.putText(frame, label, (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_clr, 1)

    # ── Main Loop ─────────────────────────────────────────────────────────────

    def _process(self) -> None:
        if not self._camera_active:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        height, width, _ = frame.shape

        # Pre-process
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = self._build_mask(hsv)

        target_found = False
        rx, ry       = self._last_target

        if mask is None:
            # Unknown color param — drain confidence
            self._confidence = max(0, self._confidence - self.DECAY_RATE)
        else:
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Keep only contours that pass all block criteria
            valid = [c for c in contours if self._is_valid_block(c)]

            if valid:
                # Select the largest valid contour as the primary target
                best = max(valid, key=cv2.contourArea)
                self._confidence = min(
                    self.MAX_CONFIDENCE, self._confidence + 1)

                M = cv2.moments(best)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    is_locked = self._confidence >= self.MAX_CONFIDENCE

                    if is_locked:
                        temp_rx, temp_ry = self._px_to_meters(cX, cY, width, height)
                        # Reachability Gate: max arm reach ≈ L1+L2 = 0.213 m from base.
                        # Camera is ~2-3 cm from base, so effective gate = 0.20 m
                        reach = math.sqrt(temp_rx**2 + temp_ry**2)
                        if reach > 0.20:
                            self._robot_state_str = "OUT OF REACH"
                            self._draw_target_annotation(frame, best, cX, cY, self._confidence, False, over_limit=True)
                            target_found = False
                        else:
                            rx, ry = temp_rx, temp_ry
                            self._last_target = (rx, ry)
                            target_found = True
                            self._draw_target_annotation(frame, best, cX, cY, self._confidence, True, over_limit=False)
                    else:
                        self._draw_target_annotation(frame, best, cX, cY, self._confidence, False, over_limit=False)
            else:
                # No valid block found — decay confidence
                self._confidence = max(0, self._confidence - self.DECAY_RATE)

        # Draw fixed HUD overlay
        if self._robot_state_str == "OUT OF REACH":
            self._draw_hud(frame, self._confidence, target_found, 0.0, 0.0)
        else:
            self._draw_hud(frame, self._confidence, target_found, rx, ry)

        # Publish target only when fully confident
        if target_found:
            pt = Point()
            pt.x = rx
            pt.y = ry
            pt.z = self.get_parameter("table_z").value
            self._target_pub.publish(pt)

        # Always publish the annotated frame
        self._image_pub.publish(
            self._bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        
        # Show live UI window if requested
        if self.get_parameter("debug_window").value:
            cv2.imshow("AI Vision - Target Detection", frame)
            cv2.waitKey(1)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = VisionPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
