import time
import math
import numpy as np

class ContactFusion:
    def __init__(self,
                 pressure_contact_threshold=0.3,
                 force_contact_threshold=1.0,
                 min_free_space_z_mm=1.0,
                 fz_bias_alpha=0.02,
                 shear_ratio_threshold=0.3,
                 moment_ratio_threshold=0.2):
        self.pressure_contact_threshold = pressure_contact_threshold
        self.force_contact_threshold = force_contact_threshold
        self.min_free_space_z_mm = min_free_space_z_mm
        self.fz_bias_alpha = fz_bias_alpha
        self.shear_ratio_threshold = shear_ratio_threshold
        self.moment_ratio_threshold = moment_ratio_threshold
        self.fz_bias = 0.0
        self.last_ts = None

    def compute_feedback(self,
                         use_pressure,
                         use_force,
                         get_pressure_fn,
                         get_force_fn,
                         robot_pose,
                         z_disp_mm):
        ts = time.time()
        self.last_ts = ts

        # Pressure: 0 valid, None => no data
        pressure = None
        if use_pressure:
            p = get_pressure_fn
            pressure = p if p is not None else None

        # Parse force sensor output
        Fx=Fy=Fz=Tx=Ty=Tz=None
        if use_force:
            Fx, Fy, Fz, Tx, Ty, Tz = self._parse_force(get_force_fn, robot_pose)

        # Compensated Fz
        fz_comp = None
        if Fz is not None:
            fz_comp = Fz - self.fz_bias

        # Contact decision
        pressure_contact = (pressure is not None) and (pressure > self.pressure_contact_threshold)
        force_contact = (fz_comp is not None) and (fz_comp < -self.force_contact_threshold)
        contact = pressure_contact or force_contact

        # Update bias only in clear free-space
        no_pressure_contact = (pressure is None) or not pressure_contact
        far_enough = abs(z_disp_mm) > self.min_free_space_z_mm
        if (Fz is not None) and no_pressure_contact and far_enough and not force_contact:
            self.fz_bias = (1 - self.fz_bias_alpha) * self.fz_bias + self.fz_bias_alpha * Fz
            fz_comp = Fz - self.fz_bias

        # Choose feedback
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
            "pressure": pressure,
            "pressure_contact": pressure_contact,
            "F_tool": {"Fx": Fx, "Fy": Fy, "Fz": Fz},
            "T_tool": {"Tx": Tx, "Ty": Ty, "Tz": Tz},
            "fz_bias": self.fz_bias,
            "fz_comp": fz_comp,
            "force_contact": force_contact,
            "contact": contact,
            "Ft": Ft,
            "M": M,
            "shear_ratio": shear_ratio,
            "moment_ratio": moment_ratio,
            "flags": {"use_pressure": use_pressure, "use_force": use_force},
        }
        return force_feedback, contact, diag

    # --- Helpers ---

    def _parse_force(self, get_force_fn, robot_pose):
        """
        Accepts various formats from get_force_sensor():
          - scalar -> tool-frame Fz
          - len==3 -> tool-frame (Fx,Fy,Fz)
          - len==6 -> base-frame (Fx,Fy,Fz,Tx,Ty,Tz) transformed to tool-frame
          - dicts with keys 'Fx','Fy','Fz','Tx','Ty','Tz' (base or tool depending on extra 'frame' key)
        """
        try:
            v = get_force_fn
        except Exception:
            return None, None, None, None, None, None

        if v is None:
            return None, None, None, None, None, None

        # Dict format
        if isinstance(v, dict):
            Fx = v.get("Fx"); Fy = v.get("Fy"); Fz = v.get("Fz")
            Tx = v.get("Tx"); Ty = v.get("Ty"); Tz = v.get("Tz")
            frame = v.get("frame")  # 'base' or 'tool'
            # If only Fz present, treat as tool-frame scalar
            if Fx is None and Fy is None and Tx is None and Ty is None and Tz is None and Fz is not None:
                return None, None, float(Fz), None, None, None
            # If full wrench and frame==base, transform
            if all(x is not None for x in (Fx, Fy, Fz)) and frame == "base":
                return self._to_tool_wrench(robot_pose, (Fx, Fy, Fz, Tx or 0.0, Ty or 0.0, Tz or 0.0))
            # Assume already tool-frame
            return (float(Fx) if Fx is not None else None,
                    float(Fy) if Fy is not None else None,
                    float(Fz) if Fz is not None else None,
                    float(Tx) if Tx is not None else None,
                    float(Ty) if Ty is not None else None,
                    float(Tz) if Tz is not None else None)

        # Iterable format
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

        # Scalar
        if isinstance(v, (int, float)):
            return None, None, float(v), None, None, None

        # Unknown format
        return None, None, None, None, None, None

    def _to_tool_wrench(self, robot_pose, wrench_base):
        Fx_b, Fy_b, Fz_b, Tx_b, Ty_b, Tz_b = wrench_base

        # Extract rxyz in radians
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

        Fx_t, Fy_t, Fz_t = float(F_t[0]), float(F_t[1]), float(F_t[2])
        Tx_t, Ty_t, Tz_t = float(T_t[0]), float(T_t[1]), float(T_t[2])
        return Fx_t, Fy_t, Fz_t, Tx_t, Ty_t, Tz_t