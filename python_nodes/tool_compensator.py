#!/usr/bin/env python3
"""
Tool (payload) gravity estimation + compensation for an F/T sensor WITHOUT an IMU.

Orientation comes from the robot (TF base->fr3_link8, or a PoseStamped topic)
instead of an IMU. The sensor's unknown mounting rotation is absorbed by the model.

Model (static, quasi-static motion only):
    F  = A @ g_f + F0              g_f = R_base_link^T @ [0, 0, -g]   (gravity in link frame)
    T  = r x (A @ g_f) + T0        r   = tool centre of mass in the sensor frame
  A (3x3) = m * R_sensor_from_link (sign convention of the sensor is absorbed too).
  F0, T0 = sensor biases.

Workflow
  1. ros2 run <pkg> tool_compensator.py        (or: python3 tool_compensator.py)
  2. Move robot to a pose, keep it still, then:
       ros2 service call /tool_compensator/record_pose std_srvs/srv/Trigger
     Repeat for ~15-20 poses with tilts about at least two different axes
     (rotating only about the vertical axis tells the fit nothing).
  3. ros2 service call /tool_compensator/fit std_srvs/srv/Trigger
     -> prints mass estimate + diagnostics and saves the YAML (params_file).
  4. Compensated wrench is now published on ~output_topic.
  5. Optional, before each task, with nothing touching the tool:
       ros2 service call /tool_compensator/tare std_srvs/srv/Trigger

Other services: /tool_compensator/clear  (drop recorded poses)

Assumes: wrench topic is geometry_msgs/WrenchStamped, base frame is gravity-aligned
(z up, robot base level), and the sensor is rigidly attached to the link used.
Use the same orientation source (TF or topic) for estimation and compensation.
"""

import os
import threading
import time

import numpy as np
import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


# ----------------------------------------------------------------------------
# Pure math helpers
# ----------------------------------------------------------------------------
def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def fit_model(G, F, T, g):
    """G, F, T: Nx3 arrays (gravity vector in link frame [m/s^2], force, torque)."""
    N = len(G)
    X = np.hstack([G / g, np.ones((N, 1))])
    coef, *_ = np.linalg.lstsq(X, F, rcond=None)          # 4x3
    A = coef[:3].T / g
    F0 = coef[3]
    sv = np.linalg.svd(X, compute_uv=False)
    cond = sv[0] / max(sv[-1], 1e-12)

    Fg = G @ A.T                                           # gravity force in sensor frame
    F_res = F - (Fg + F0)

    # T_i = -skew(Fg_i) r + T0
    M = np.vstack([np.hstack([-skew(f), np.eye(3)]) for f in Fg])
    sol, *_ = np.linalg.lstsq(M, T.reshape(-1), rcond=None)
    r, T0 = sol[:3], sol[3:]
    T_res = T - (np.cross(r, Fg) + T0)

    a_sv = np.linalg.svd(A, compute_uv=False)
    mass = float(a_sv.mean())
    return {
        'A': A, 'F0': F0, 'r': r, 'T0': T0,
        'mass': mass,
        'A_sv_spread': float((a_sv.max() - a_sv.min()) / max(mass, 1e-12)),
        'cond': float(cond),
        'F_rms': float(np.sqrt(np.mean(F_res ** 2))),
        'T_rms': float(np.sqrt(np.mean(T_res ** 2))),
    }


# ----------------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------------
class ToolCompensator(Node):

    COND_WARN = 20.0     # heuristic: pose set is getting poorly spread
    COND_FAIL = 200.0    # heuristic: pose set clearly lacks orientation variety

    def __init__(self):
        super().__init__('tool_compensator')

        self.declare_parameter('wrench_topic', '/bota_ft/wrench')
        self.declare_parameter('output_topic', '/bota_ft/wrench_compensated')
        self.declare_parameter('pose_source', 'tf')           # 'tf' or 'topic'
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('link_frame', 'fr3_link8')
        self.declare_parameter('pose_topic', '/current_pose')  # PoseStamped, pose of the link in base
        self.declare_parameter('params_file', '/home/ergodic_group/Matej/pixi_franka_ros2/config/tool_compensation.yaml')
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('settle_delay', 1.0)           # s to wait after record_pose call
        self.declare_parameter('record_duration', 2.0)        # s averaged per pose
        self.declare_parameter('rot_tol_deg', 1.0)            # max orientation change while recording
        self.declare_parameter('min_samples', 8)              # min poses for fit
        self.declare_parameter('tare_duration', 1.0)
        self.declare_parameter('load_on_start', True)

        p = self.get_parameter
        self.pose_source = p('pose_source').value
        self.base_frame = p('base_frame').value
        self.link_frame = p('link_frame').value
        self.g = float(p('gravity').value)
        self.params_file = os.path.expanduser(p('params_file').value)
        self.g_base = np.array([0.0, 0.0, -self.g])

        self._lock = threading.Lock()
        self._rec = None
        self._R_topic = None
        self.samples = []                    # list of dicts {g_f, F, T}
        self.model = None
        self.bias = np.zeros(6)              # tare bias [F, T] on top of the model

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.pose_source == 'topic':
            self.create_subscription(PoseStamped, p('pose_topic').value,
                                     self._pose_cb, qos_profile_sensor_data,
                                     callback_group=cb)
        self.create_subscription(WrenchStamped, p('wrench_topic').value,
                                 self._wrench_cb, qos_profile_sensor_data,
                                 callback_group=cb)
        self.pub = self.create_publisher(WrenchStamped, p('output_topic').value, 10)

        self.create_service(Trigger, '~/record_pose', self._record_pose_cb, callback_group=cb)
        self.create_service(Trigger, '~/fit', self._fit_cb, callback_group=cb)
        self.create_service(Trigger, '~/clear', self._clear_cb, callback_group=cb)
        self.create_service(Trigger, '~/tare', self._tare_cb, callback_group=cb)

        if p('load_on_start').value and os.path.isfile(self.params_file):
            if self._load_model(self.params_file):
                self.get_logger().info(
                    f'Loaded tool model from {self.params_file} '
                    f'(mass ~ {self.model["mass"]:.3f} kg). Compensating.')
        if self.model is None:
            self.get_logger().info('No tool model loaded. Use record_pose / fit to estimate one.')

    # ---------------------------------------------------------------- orientation
    def _pose_cb(self, msg):
        q = msg.pose.orientation
        self._R_topic = quat_to_R(q.x, q.y, q.z, q.w)

    def _get_R(self):
        """Rotation of the link expressed in the base frame, or None."""
        if self.pose_source == 'topic':
            return self._R_topic
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.link_frame, Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return quat_to_R(q.x, q.y, q.z, q.w)

    # ---------------------------------------------------------------- model
    def _model_wrench(self, f, t, R):
        """Remove gravity + bias according to the fitted model (without tare)."""
        m = self.model
        g_f = R.T @ self.g_base
        Fg = m['A'] @ g_f
        return f - Fg - m['F0'], t - (np.cross(m['r'], Fg) + m['T0'])

    def _save_model(self, path):
        m = self.model
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        data = {
            'A': m['A'].ravel().tolist(), 'F0': m['F0'].tolist(),
            'r': m['r'].tolist(), 'T0': m['T0'].tolist(),
            'mass': m['mass'], 'base_frame': self.base_frame,
            'link_frame': self.link_frame, 'pose_source': self.pose_source,
            'n_samples': len(self.samples),
        }
        with open(path, 'w') as fh:
            yaml.safe_dump(data, fh)

    def _load_model(self, path):
        try:
            with open(path) as fh:
                d = yaml.safe_load(fh)
            A = np.array(d['A'], dtype=float).reshape(3, 3)
            self.model = {'A': A, 'F0': np.array(d['F0'], dtype=float),
                          'r': np.array(d['r'], dtype=float),
                          'T0': np.array(d['T0'], dtype=float),
                          'mass': float(d.get('mass', np.linalg.svd(A, compute_uv=False).mean()))}
            self.bias = np.zeros(6)
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Failed to load {path}: {e}')
            return False

    # ---------------------------------------------------------------- wrench stream
    def _wrench_cb(self, msg):
        w = msg.wrench
        f = np.array([w.force.x, w.force.y, w.force.z])
        t = np.array([w.torque.x, w.torque.y, w.torque.z])
        R = self._get_R()
        if R is None:
            self.get_logger().warn('Link orientation not available yet.',
                                   throttle_duration_sec=2.0)
            return

        with self._lock:
            rec = self._rec
            if rec is not None and not rec['done']:
                now = self.get_clock().now().nanoseconds * 1e-9
                if rec['t0'] is None:
                    rec['t0'] = now
                rec['F'].append(f)
                rec['T'].append(t)
                rec['R'].append(R)
                if now - rec['t0'] >= rec['dur']:
                    rec['done'] = True
                    rec['ev'].set()

        if self.model is not None:
            fc, tc = self._model_wrench(f, t, R)
            out = WrenchStamped()
            out.header = msg.header
            out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = (fc - self.bias[:3]).tolist()
            out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = (tc - self.bias[3:]).tolist()
            self.pub.publish(out)

    def _collect(self, duration):
        """Block (in a service thread) until `duration` s of data were recorded."""
        ev = threading.Event()
        with self._lock:
            if self._rec is not None:
                return None, 'another recording is in progress'
            self._rec = {'F': [], 'T': [], 'R': [], 't0': None,
                         'dur': duration, 'ev': ev, 'done': False}
        got = ev.wait(timeout=duration + 3.0)
        with self._lock:
            rec, self._rec = self._rec, None
        if not got or len(rec['F']) < 10:
            return None, ('no/too little data: check wrench topic and that the '
                          'orientation source (TF / pose topic) is available')
        return {k: np.array(rec[k]) for k in ('F', 'T', 'R')}, ''

    # ---------------------------------------------------------------- services
    def _record_pose_cb(self, _req, res):
        time.sleep(float(self.get_parameter('settle_delay').value))
        data, err = self._collect(float(self.get_parameter('record_duration').value))
        if data is None:
            res.success, res.message = False, err
            return res

        Rs = data['R']
        R0 = Rs[0]
        max_ang = max(np.arccos(np.clip((np.trace(R0.T @ Ri) - 1) / 2, -1.0, 1.0))
                      for Ri in Rs)
        tol = np.deg2rad(float(self.get_parameter('rot_tol_deg').value))
        if max_ang > tol:
            res.success = False
            res.message = (f'Robot moved while recording ({np.rad2deg(max_ang):.2f} deg > '
                           f'{np.rad2deg(tol):.2f} deg). Pose NOT recorded, try again.')
            return res

        g_f = np.mean([Ri.T @ self.g_base for Ri in Rs], axis=0)
        s = {'g_f': g_f, 'F': data['F'].mean(axis=0), 'T': data['T'].mean(axis=0)}
        self.samples.append(s)
        res.success = True
        res.message = (f'Recorded pose #{len(self.samples)} '
                       f'(F std {data["F"].std(axis=0).max():.3f} N, '
                       f'T std {data["T"].std(axis=0).max():.4f} Nm)')
        self.get_logger().info(res.message)
        return res

    def _fit_cb(self, _req, res):
        n = len(self.samples)
        min_n = int(self.get_parameter('min_samples').value)
        if n < min_n:
            res.success, res.message = False, f'Need at least {min_n} poses, have {n}.'
            return res

        G = np.array([s['g_f'] for s in self.samples])
        F = np.array([s['F'] for s in self.samples])
        T = np.array([s['T'] for s in self.samples])
        out = fit_model(G, F, T, self.g)

        if out['cond'] > self.COND_FAIL:
            res.success = False
            res.message = (f'Poses lack orientation variety (cond={out["cond"]:.0f}). '
                           'Add poses tilted about more than one axis.')
            return res

        self.model = out
        self.bias = np.zeros(6)
        self._save_model(self.params_file)

        warns = []
        if out['cond'] > self.COND_WARN:
            warns.append(f'pose spread is marginal (cond={out["cond"]:.1f})')
        if out['A_sv_spread'] > 0.1:
            warns.append(f'A is not close to m*rotation (spread {100 * out["A_sv_spread"]:.0f}%): '
                         'poses were not still, wrong frame/base not level, or sensor not rigid')
        res.success = True
        res.message = (f'Fit OK with {n} poses. mass~{out["mass"]:.3f} kg, '
                       f'CoM in sensor frame r={np.round(out["r"], 4).tolist()} m, '
                       f'residual RMS F={out["F_rms"]:.3f} N, T={out["T_rms"]:.4f} Nm. '
                       f'Saved to {self.params_file}.'
                       + (' WARNINGS: ' + '; '.join(warns) if warns else ''))
        self.get_logger().info(res.message)
        return res

    def _clear_cb(self, _req, res):
        self.samples.clear()
        res.success, res.message = True, 'Recorded poses cleared (current model kept).'
        return res

    def _tare_cb(self, _req, res):
        if self.model is None:
            res.success, res.message = False, 'No model yet.'
            return res
        data, err = self._collect(float(self.get_parameter('tare_duration').value))
        if data is None:
            res.success, res.message = False, err
            return res
        comp = [np.hstack(self._model_wrench(f, t, R))
                for f, t, R in zip(data['F'], data['T'], data['R'])]
        self.bias = np.mean(comp, axis=0)
        res.success = True
        res.message = f'Tared. Residual bias F={np.round(self.bias[:3], 3).tolist()} N, ' \
                      f'T={np.round(self.bias[3:], 4).tolist()} Nm'
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = ToolCompensator()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
