#!/usr/bin/env python3
"""
aruco_tf_detector.py

Detects a single ArUco marker or a ChArUco board in a camera image and
broadcasts its pose to tf as:  <image frame_id>  ->  <marker_frame>

Designed as the "tracking system" for easy_handeye2 (eye-on-base).
Requires OpenCV >= 4.7 (new cv2.aruco API). No cv_bridge needed.

Example (ChArUco):
  python aruco_tf_detector.py --ros-args \
    -r __node:=cam1_detector \
    -p mode:=charuco \
    -p image_topic:=/cam1/cam1/color/image_raw \
    -p camera_info_topic:=/cam1/cam1/color/camera_info \
    -p marker_frame:=cam1_marker \
    -p dictionary:=DICT_5X5_100 \
    -p squares_x:=7 -p squares_y:=5 \
    -p square_length:=0.04 -p marker_length:=0.03

Example (single marker):
  python aruco_tf_detector.py --ros-args \
    -p mode:=aruco -p marker_id:=0 -p marker_size:=0.15 \
    -p dictionary:=DICT_4X4_50 \
    -p image_topic:=... -p camera_info_topic:=... -p marker_frame:=cam1_marker
"""

import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def image_msg_to_gray(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to a grayscale uint8 array (no cv_bridge)."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if msg.encoding not in channels:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    ch = channels[msg.encoding]
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    # rows may be padded: reshape by step, then crop to the real width
    img = buf.reshape(msg.height, msg.step)[:, : msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch) if ch > 1 else img
    code = {
        "rgb8": cv2.COLOR_RGB2GRAY,
        "bgr8": cv2.COLOR_BGR2GRAY,
        "rgba8": cv2.COLOR_RGBA2GRAY,
        "bgra8": cv2.COLOR_BGRA2GRAY,
    }.get(msg.encoding)
    return np.ascontiguousarray(img) if code is None else cv2.cvtColor(img, code)


def rotation_to_quaternion(R: np.ndarray):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoTfDetector(Node):
    def __init__(self):
        super().__init__("aruco_tf_detector")

        # ---- parameters ----
        self.declare_parameter("mode", "charuco")  # "aruco" | "charuco"
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("marker_frame", "marker")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        # single marker
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.10)  # side length [m], black square
        # charuco board
        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length", 0.04)  # [m]
        self.declare_parameter("marker_length", 0.03)  # [m]
        self.declare_parameter("legacy_pattern", False)
        self.declare_parameter("min_charuco_corners", 6)
        # quality gate + debug
        self.declare_parameter("max_reproj_error_px", 1.8)
        self.declare_parameter("publish_debug_image", True)

        p = self.get_parameter
        self.mode = p("mode").value
        self.marker_frame = p("marker_frame").value
        self.marker_id = int(p("marker_id").value)
        self.marker_size = float(p("marker_size").value)
        self.min_corners = int(p("min_charuco_corners").value)
        self.max_err = float(p("max_reproj_error_px").value)
        self.publish_debug = bool(p("publish_debug_image").value)

        if self.mode not in ("aruco", "charuco"):
            raise ValueError("mode must be 'aruco' or 'charuco'")

        # ---- OpenCV detector setup ----
        dict_name = p("dictionary").value
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f"Unknown dictionary '{dict_name}'")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

        det_params = cv2.aruco.DetectorParameters()
        det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det_params.cornerRefinementWinSize = 5
        det_params.cornerRefinementMaxIterations = 50
        det_params.cornerRefinementMinAccuracy = 0.01

        if self.mode == "aruco":
            self.detector = cv2.aruco.ArucoDetector(dictionary, det_params)
            s = self.marker_size / 2.0
            # order matches OpenCV corner order: TL, TR, BR, BL
            self.marker_obj_pts = np.array(
                [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32
            )
        else:
            self.board = cv2.aruco.CharucoBoard(
                (int(p("squares_x").value), int(p("squares_y").value)),
                float(p("square_length").value),
                float(p("marker_length").value),
                dictionary,
            )
            if bool(p("legacy_pattern").value):
                self.board.setLegacyPattern(True)
            self.charuco_detector = cv2.aruco.CharucoDetector(
                self.board, cv2.aruco.CharucoParameters(), det_params
            )

        # ---- ROS plumbing ----
        self.K = None
        self.D = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo, p("camera_info_topic").value, self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, p("image_topic").value, self.on_image, qos_profile_sensor_data
        )
        self.debug_pub = self.create_publisher(Image, "/camera/cam1/color/debug_image", 1)

        self.get_logger().info(
            f"Mode '{self.mode}', publishing tf -> '{self.marker_frame}'"
        )

    # ------------------------------------------------------------------
    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = (
            np.array(msg.d, dtype=np.float64)
            if len(msg.d) > 0
            else np.zeros(5, dtype=np.float64)
        )

    def on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn(
                "Waiting for camera_info...", throttle_duration_sec=5.0
            )
            return
        try:
            gray = image_msg_to_gray(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        result = (
            self.estimate_marker(gray)
            if self.mode == "aruco"
            else self.estimate_charuco(gray)
        )

        debug = None
        if self.publish_debug:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if result is None:
            self.get_logger().warn(
                "No valid detection", throttle_duration_sec=5.0
            )
        else:
            rvec, tvec, err = result
            self.broadcast(msg, rvec, tvec)
            if debug is not None:
                cv2.drawFrameAxes(debug, self.K, self.D, rvec, tvec, 0.05, 3)
                cv2.putText(
                    debug, f"reproj {err:.2f}px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
                )
        if debug is not None:
            self.publish_debug_image(msg, debug)

    # ------------------------------------------------------------------
    def estimate_marker(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        idx = np.where(ids.flatten() == self.marker_id)[0]
        if len(idx) == 0:
            return None
        img_pts = corners[int(idx[0])].reshape(4, 2).astype(np.float32)
        # IPPE_SQUARE returns both ambiguous solutions, best (lowest error) first
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            self.marker_obj_pts, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if n < 1:
            return None
        rvec, tvec = rvecs[0], tvecs[0]
        err = self.reproj_error(self.marker_obj_pts, img_pts, rvec, tvec)
        return self.gate(rvec, tvec, err)

    def estimate_charuco(self, gray):
        ch_corners, ch_ids, _, _ = self.charuco_detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < self.min_corners:
            return None
        obj_pts, img_pts = self.board.matchImagePoints(ch_corners, ch_ids)
        if obj_pts is None or len(obj_pts) < self.min_corners:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None
        err = self.reproj_error(obj_pts, img_pts, rvec, tvec)
        return self.gate(rvec, tvec, err)

    def reproj_error(self, obj_pts, img_pts, rvec, tvec):
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.D)
        d = proj.reshape(-1, 2) - img_pts.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))

    def gate(self, rvec, tvec, err):
        """Reject poses with large reprojection error (bad samples hurt calibration)."""
        if err > self.max_err:
            self.get_logger().warn(
                f"Rejected pose, reprojection error {err:.2f}px > {self.max_err}px",
                throttle_duration_sec=2.0,
            )
            return None
        return rvec, tvec, err

    # ------------------------------------------------------------------
    def broadcast(self, img_msg, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_to_quaternion(R)
        t = TransformStamped()
        t.header.stamp = img_msg.header.stamp
        t.header.frame_id = img_msg.header.frame_id
        t.child_frame_id = self.marker_frame
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def publish_debug_image(self, src_msg, bgr):
        out = Image()
        out.header = src_msg.header
        out.height, out.width = bgr.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.debug_pub.publish(out)


def main():
    rclpy.init(args=sys.argv)
    node = ArucoTfDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
