import numpy as np
from behaviors import StateBehavior
from scipy.spatial.transform import Rotation as R

class TrackMarkerSB(StateBehavior):
    """Track the board: align EE z with the marker z-axis and hover at a standoff.

    Assumed on node:
      node.get_marker_pose() -> (R_base_marker: scipy Rotation, p_base_marker: np.ndarray(3)) | None
          (None if TF is missing or stale; base frame = the frame of target_pose)
      node.standoff, node.linear_speed, node.angular_speed, node.dt
      node.fsm.track_done()
    """

    ALPHA = 0.2            # low-pass factor (0..1), lower = smoother
    POS_EPS = 0.001        # m
    ANG_EPS = np.deg2rad(0.5)
    SETTLE_TICKS = 25      # consecutive converged ticks required
    MAX_JUMP = 0.05        # m, reject marker samples that jump more than this
    MAX_Z_JUMP = np.deg2rad(30)  # reject z-axis flips (planar ambiguity)

    def on_enter(self, node):
        node.get_logger().info("Tracking marker...")
        self.z_f = None      # filtered marker z-axis (unit vector, into the board)
        self.p_f = None      # filtered marker position
        self.settled = 0

    @staticmethod
    def _min_rotation(a, b):
        """Smallest rotation taking unit vector a onto unit vector b."""
        axis = np.cross(a, b)
        s, c = np.linalg.norm(axis), np.dot(a, b)
        if s < 1e-9:
            if c > 0:
                return R.identity()
            perp = np.cross(a, [1.0, 0.0, 0.0])
            if np.linalg.norm(perp) < 1e-3:
                perp = np.cross(a, [0.0, 1.0, 0.0])
            return R.from_rotvec(np.pi * perp / np.linalg.norm(perp))
        return R.from_rotvec(axis / s * np.arctan2(s, c))

    def _update_filter(self, node):
        """Fold the newest marker measurement into the filter. Returns False if no usable data."""
        meas = node.get_marker_pose()
        if meas is None:
            return self.z_f is not None   # keep going on filtered value after dropout

        R_m, p_m = meas
        z_m = R_m.apply([0.0, 0.0, 1.0])

        if self.z_f is None:              # first sample initializes the filter
            self.z_f, self.p_f = z_m, p_m
            return True

        # outlier / flip rejection
        ang = np.arccos(np.clip(np.dot(self.z_f, z_m), -1.0, 1.0))
        if np.linalg.norm(p_m - self.p_f) > self.MAX_JUMP or ang > self.MAX_Z_JUMP:
            node.get_logger().warn("Rejected marker sample (jump/flip)", throttle_duration_sec=1.0)
            return True

        a = self.ALPHA
        self.p_f = (1 - a) * self.p_f + a * p_m
        z = (1 - a) * self.z_f + a * z_m   # filter the vector, not the quaternion
        self.z_f = z / np.linalg.norm(z)
        return True

    def tick(self, node):
        if not self._update_filter(node):
            return  # never seen the marker yet: hold current target

        # --- desired pose ---
        z_des = self.z_f
        p_des = self.p_f - node.standoff * z_des   # hover on the outward side of the board

        # --- current commanded pose ---
        o = node.target_pose.pose.orientation
        pos = node.target_pose.pose.position
        R_curr = R.from_quat([o.x, o.y, o.z, o.w])
        p_curr = np.array([pos.x, pos.y, pos.z])
        z_curr = R_curr.apply([0.0, 0.0, 1.0])

        # --- orientation step: minimal rotation, keeps current yaw ---
        rot_err = self._min_rotation(z_curr, z_des)
        angle_err = rot_err.magnitude()
        ratio = min(node.angular_speed * node.dt / angle_err, 1.0) if angle_err > 1e-9 else 1.0
        R_next = R.from_rotvec(rot_err.as_rotvec() * ratio) * R_curr
        q_next = R_next.as_quat()

        # --- position step ---
        pos_err_vec = p_des - p_curr
        pos_err = np.linalg.norm(pos_err_vec)
        step = node.linear_speed * node.dt
        p_next = p_des if pos_err <= step else p_curr + pos_err_vec / pos_err * step

        # --- write target ---
        node.target_pose.pose.orientation.x = float(q_next[0])
        node.target_pose.pose.orientation.y = float(q_next[1])
        node.target_pose.pose.orientation.z = float(q_next[2])
        node.target_pose.pose.orientation.w = float(q_next[3])
        node.target_pose.pose.position.x = float(p_next[0])
        node.target_pose.pose.position.y = float(p_next[1])
        node.target_pose.pose.position.z = float(p_next[2])

        # --- convergence: must hold for several consecutive ticks ---
        if angle_err <= self.ANG_EPS and pos_err <= self.POS_EPS:
            self.settled += 1
            if self.settled >= self.SETTLE_TICKS:
                node.fsm.track_done()
        else:
            self.settled = 0

    def on_leave(self, node):
        node.get_logger().info("Tracking complete.")