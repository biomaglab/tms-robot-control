import time
import math
import numpy as np


class EMAFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.y = None

    def reset(self):
        self.y = None

    def update(self, x):
        if x is None:
            return self.y
        if self.y is None:
            self.y = float(x)
        else:
            self.y = (1 - self.alpha) * self.y + self.alpha * float(x)
        return self.y


class MedianFilterN:
    def __init__(self, N=5):
        self.N = max(1, int(N))
        self.buf = []

    def reset(self):
        self.buf = []

    def update(self, x):
        if x is None:
            return None
        self.buf.append(float(x))
        if len(self.buf) > self.N:
            self.buf.pop(0)
        return float(sorted(self.buf)[len(self.buf)//2])


class RateLimiter:
    def __init__(self, max_delta_per_update):
        self.max_delta = float(max_delta_per_update)
        self.last = None

    def reset(self):
        self.last = None

    def limit(self, target):
        if target is None:
            return self.last
        t = float(target)
        if self.last is None:
            self.last = t
            return t
        delta = t - self.last
        if delta > self.max_delta:
            self.last += self.max_delta
        elif delta < -self.max_delta:
            self.last -= self.max_delta
        else:
            self.last = t
        return self.last


class ContactFusionRLS:
    def __init__(self,
                 pressure_contact_threshold=0.3,
                 pressure_release_threshold=0.2,
                 force_contact_threshold=1.0,
                 force_release_threshold=0.6,
                 min_free_space_z_mm=0.5,
                 ema_alpha=0.02,
                 rls_lambda=0.997,
                 rls_delta=2000.0,
                 shear_ratio_threshold=0.3,
                 moment_ratio_threshold=0.2,
                 lp_alpha_force=0.1,
                 lp_alpha_pressure=0.2,
                 median_N=5):
        # thresholds + hysteresis
        self.p_on = pressure_contact_threshold
        self.p_off = pressure_release_threshold
        self.f_on = force_contact_threshold
        self.f_off = force_release_threshold
        self.min_free_z = min_free_space_z_mm

        # EMA fallback
        self.ema_alpha = ema_alpha
        self.fz_bias_ema = 0.0

        # RLS model
        self.n_features = 7
        self.theta = np.zeros((self.n_features, 1))
        self.P = np.eye(self.n_features) * rls_delta
        self.rls_lambda = rls_lambda
        self.samples = 0

        # diagnostics thresholds
        self.shear_ratio_threshold = shear_ratio_threshold
        self.moment_ratio_threshold = moment_ratio_threshold

        # filters
        self.lp_force = EMAFilter(lp_alpha_force)
        self.lp_pressure = EMAFilter(lp_alpha_pressure)
        self.med_force = MedianFilterN(median_N)
        self.med_pressure = MedianFilterN(median_N)

        # state
        self.last_ts = None
        self.contact_state = False

    def compute_feedback(self,
                         use_pressure,
                         use_force,
                         get_pressure_fn,
                         get_force_fn,
                         robot_pose,
                         z_disp_mm):
        ts = time.time()
        self.last_ts = ts

        # Pressure
        pressure_raw = None
        if use_pressure:
            try:
                p = get_pressure_fn()
                pressure_raw = p if p is not None else None
            except Exception:
                pressure_raw = None

        # Force/torque
        Fx = Fy = Fz = Tx = Ty = Tz = None
        if use_force:
            Fx, Fy, Fz, Tx, Ty, Tz = self._parse_force(get_force_fn, robot_pose)

        # Predict bias from pose (RLS), fallback to EMA if not trained yet
        fz_bias_pred = 0.0
        if Fz is not None:
            phi = self._features_from_pose(robot_pose)
            fz_bias_pred = float(phi.T @ self.theta) if self.samples > 15 else self.fz_bias_ema

        # Compensated Fz
        fz_comp_raw = None
        if Fz is not None:
            fz_comp_raw = Fz - fz_bias_pred

        # Median + low-pass filtering
        fz_comp_med = self.med_force.update(fz_comp_raw) if fz_comp_raw is not None else None
        fz_comp = self.lp_force.update(fz_comp_med) if fz_comp_med is not None else None

        pressure_med = self.med_pressure.update(pressure_raw) if pressure_raw is not None else None
        pressure = self.lp_pressure.update(pressure_med) if pressure_med is not None else None

        # Hysteretic contact decision
        pressure_contact = False
        force_contact = False

        if pressure is not None:
            pressure_contact = (pressure > self.p_on) if not self.contact_state else (pressure > self.p_off)

        if fz_comp is not None:
            # compression along tool Z is negative
            force_contact = (fz_comp < -self.f_on) if not self.contact_state else (fz_comp < -self.f_off)

        self.contact_state = pressure_contact or force_contact

        # Bias update in free-space (on raw Fz to model true preload)
        no_pressure_contact = (pressure is None) or (pressure <= self.p_off)
        small_normal = (fz_comp is None) or (abs(fz_comp) < 0.5 * self.f_on)
        far_enough = abs(z_disp_mm) >= self.min_free_z

        if (Fz is not None) and no_pressure_contact and small_normal and far_enough and not self.contact_state:
            phi = self._features_from_pose(robot_pose)
            Pphi = self.P @ phi
            gain_den = self.rls_lambda + (phi.T @ Pphi)[0, 0]
            K = Pphi / gain_den
            y_hat = float(phi.T @ self.theta)
            err = Fz - y_hat
            self.theta = self.theta + K * err
            self.P = (self.P - K @ phi.T @ self.P) / self.rls_lambda
            self.samples += 1

            self.fz_bias_ema = (1 - self.ema_alpha) * self.fz_bias_ema + self.ema_alpha * Fz

            # Refresh prediction and compensation
            fz_bias_pred = float(phi.T @ self.theta)
            fz_comp_raw = Fz - fz_bias_pred
            fz_comp_med = self.med_force.update(fz_comp_raw)
            fz_comp = self.lp_force.update(fz_comp_med)

        # Choose feedback (prefer pressure if valid contact; else Fz)
        if use_pressure and use_force:
            if pressure is not None:
                force_feedback = fz_comp if ((not pressure_contact) and force_contact) else pressure
            else:
                force_feedback = fz_comp
        elif use_pressure:
            force_feedback = pressure
        elif use_force:
            force_feedback = fz_comp
        else:
            force_feedback = None

        # Diagnostics
        Ft = math.hypot(Fx, Fy) if (Fx is not None and Fy is not None) else None
        M = math.sqrt(Tx*Tx + Ty*Ty + Tz*Tz) if (Tx is not None and Ty is not None and Tz is not None) else None
        shear_ratio = (Ft / abs(fz_comp)) if (Ft is not None and fz_comp is not None and abs(fz_comp) > 1e-6) else None
        moment_ratio = (M / abs(fz_comp)) if (M is not None and fz_comp is not None and abs(fz_comp) > 1e-6) else None

        diag = {
            "ts": ts,
            "pressure_raw": pressure_raw,
            "pressure": pressure,
            "pressure_contact": pressure_contact,
            "F_tool": {"Fx": Fx, "Fy": Fy, "Fz": Fz},
            "fz_bias_pred": fz_bias_pred,
            "fz_bias_ema": self.fz_bias_ema,
            "fz_comp_raw": fz_comp_raw,
            "fz_comp": fz_comp,
            "force_contact": force_contact,
            "contact": self.contact_state,
            "Ft": Ft,
            "M": M,
            "shear_ratio": shear_ratio,
            "moment_ratio": moment_ratio,
            "rls_samples": self.samples,
        }
        return force_feedback, self.contact_state, diag

    # --- helpers ---

    def _features_from_pose(self, robot_pose):
        x = robot_pose.get("x") if isinstance(robot_pose, dict) else getattr(robot_pose, "x", 0.0)
        y = robot_pose.get("y") if isinstance(robot_pose, dict) else getattr(robot_pose, "y", 0.0)
        z = robot_pose.get("z") if isinstance(robot_pose, dict) else getattr(robot_pose, "z", 0.0)
        rx = robot_pose.get("rx") if isinstance(robot_pose, dict) else getattr(robot_pose, "rx", 0.0)
        ry = robot_pose.get("ry") if isinstance(robot_pose, dict) else getattr(robot_pose, "ry", 0.0)
        rz = robot_pose.get("rz") if isinstance(robot_pose, dict) else getattr(robot_pose, "rz", 0.0)
        return np.array([x, y, z, rx, ry, rz, 1.0], dtype=float).reshape(-1, 1)

    def _parse_force(self, get_force_fn, robot_pose):
        try:
            v = get_force_fn()
        except Exception:
            return None, None, None, None, None, None
        if v is None:
            return None, None, None, None, None, None

        if isinstance(v, dict):
            Fx=v.get("Fx"); Fy=v.get("Fy"); Fz=v.get("Fz")
            Tx=v.get("Tx"); Ty=v.get("Ty"); Tz=v.get("Tz")
            frame=v.get("frame")
            if Fx is None and Fy is None and Tx is None and Ty is None and Tz is None and Fz is not None:
                return None, None, float(Fz), None, None, None
            if all(x is not None for x in (Fx, Fy, Fz)) and frame == "base":
                return self._to_tool_wrench(robot_pose, (Fx, Fy, Fz, Tx or 0.0, Ty or 0.0, Tz or 0.0))
            return (float(Fx) if Fx is not None else None,
                    float(Fy) if Fy is not None else None,
                    float(Fz) if Fz is not None else None,
                    float(Tx) if Tx is not None else None,
                    float(Ty) if Ty is not None else None,
                    float(Tz) if Tz is not None else None)

        if hasattr(v, "__len__"):
            n = len(v)
            if n == 1:
                return None, None, float(v[0]), None, None, None
            if n == 3:
                Fx, Fy, Fz = map(float, v)
                return Fx, Fy, Fz, None, None, None
            if n == 6:
                Fx_b, Fy_b, Fz_b, Tx_b, Ty_b, Tz_b = map(float, v)
                return self._to_tool_wrench(robot_pose, (Fx_b, Fy_b, Fz_b, Tx_b, Ty_b, Tz_b))

        if isinstance(v, (int, float)):
            return None, None, float(v), None, None, None

        return None, None, None, None, None, None

    def _to_tool_wrench(self, robot_pose, wrench_base):
        Fx_b, Fy_b, Fz_b, Tx_b, Ty_b, Tz_b = wrench_base
        rx = robot_pose.get("rx") if isinstance(robot_pose, dict) else getattr(robot_pose, "rx", 0.0)
        ry = robot_pose.get("ry") if isinstance(robot_pose, dict) else getattr(robot_pose, "ry", 0.0)
        rz = robot_pose.get("rz") if isinstance(robot_pose, dict) else getattr(robot_pose, "rz", 0.0)

        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)

        Rz = np.array([[cz, -sz, 0],
                       [sz,  cz, 0],
                       [ 0,   0, 1]])
        Ry = np.array([[ cy, 0, sy],
                       [  0, 1,  0],
                       [-sy, 0, cy]])
        Rx = np.array([[1,  0,   0],
                       [0, cx, -sx],
                       [0, sx,  cx]])
        R = Rz @ Ry @ Rx

        F_b = np.array([Fx_b, Fy_b, Fz_b])
        T_b = np.array([Tx_b, Ty_b, Tz_b])
        F_t = R.T @ F_b
        T_t = R.T @ T_b
        return float(F_t[0]), float(F_t[1]), float(F_t[2]), float(T_t[0]), float(T_t[1]), float(T_t[2])
