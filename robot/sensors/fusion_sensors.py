import time
import math
import numpy as np


class EMAFilter:
    def __init__(self, alpha=0.2):
        self.alpha = float(alpha)
        self.y = None

    def update(self, x):
        if x is None: return self.y
        x = float(x)
        self.y = x if self.y is None else (1 - self.alpha) * self.y + self.alpha * x
        return self.y

class MedianFilterN:
    def __init__(self, N=5):
        self.N = max(1, int(N))
        self.buf = []

    def update(self, x):
        if x is None: return None
        self.buf.append(float(x))
        if len(self.buf) > self.N: self.buf.pop(0)
        return float(sorted(self.buf)[len(self.buf)//2])


class PressureFirstContact:
    """
    Pressure drives control; F/T used only for robust safety/misalignment checks.
    Cable effects mitigated via:
      - tool-frame transform
      - filtering + hysteresis on pressure
      - slow baselines of Ft and M (EMA) while pressure is low
      - persistence + dwell time + cooldown
      - deadbands around baselines
      - optional proximity gating (only check F/T when near expected contact)
    """

    def __init__(self,
                 pressure_contact_threshold=0.30,
                 pressure_release_threshold=0.20,
                 # F/T thresholds (tool frame):
                 fz_big_contact=3.0,            # N (compression negative)
                 ft_hard_limit=6.0,             # N
                 m_hard_limit=1.0,              # Nm
                 shear_ratio_threshold=0.5,     # Ft/|Fz| diagnostic
                 moment_ratio_threshold=0.35,   # |M|/|Fz| diagnostic
                 # Filtering
                 pressure_median_N=5,
                 pressure_lp_alpha=0.2,
                 baseline_alpha=0.01,           # very slow
                 # Robustness controls
                 ft_deadband=0.5,               # N deviation allowed around baseline
                 m_deadband=0.1,                # Nm deviation allowed around baseline
                 persistence_frames=4,          # require consecutive frames
                 dwell_time_s=0.2,              # must persist at least this long
                 cooldown_s=0.5,                # after triggering, ignore new triggers briefly
                 proximity_pressure_low=0.10,   # start F/T checks only when pressure above this (near contact), use 0 to disable
                 ):
        # Pressure hysteresis
        self.p_on  = float(pressure_contact_threshold)
        self.p_off = float(pressure_release_threshold)

        # Thresholds
        self.fz_big_contact = float(fz_big_contact)
        self.ft_hard_limit  = float(ft_hard_limit)
        self.m_hard_limit   = float(m_hard_limit)
        self.shear_ratio_threshold  = float(shear_ratio_threshold)
        self.moment_ratio_threshold = float(moment_ratio_threshold)

        # Filters
        self.p_med = MedianFilterN(pressure_median_N)
        self.p_lp  = EMAFilter(pressure_lp_alpha)

        # Baselines (learned when pressure is low)
        self.Ft_base = EMAFilter(baseline_alpha)
        self.M_base  = EMAFilter(baseline_alpha)

        # Deadbands
        self.ft_deadband = float(ft_deadband)
        self.m_deadband  = float(m_deadband)

        # State
        self.contact_state = False
        self.big_contact_inst_frames = 0
        self.over_force_inst_frames = 0
        self.persistence_frames = int(persistence_frames)
        self.dwell_time_s = float(dwell_time_s)
        self.cooldown_s = float(cooldown_s)
        self.last_big_contact_start_ts = None
        self.last_trigger_ts = 0.0  # cooldown timer
        self.last_ts = None

        # Proximity gating
        self.proximity_pressure_low = float(proximity_pressure_low)

    def compute(self, use_pressure, use_force, get_pressure_fn, get_force_fn, robot_pose):
        ts = time.time()
        self.last_ts = ts

        # Read & filter pressure (0 valid)
        pressure_raw = get_pressure_fn() if use_pressure else None
        pressure_med = self.p_med.update(pressure_raw) if pressure_raw is not None else None
        pressure     = self.p_lp.update(pressure_med)   if pressure_med is not None else None

        # Pressure contact hysteresis
        if pressure is not None:
            self.contact_state = (pressure > self.p_on) if not self.contact_state else (pressure > self.p_off)

        # F/T for checks only
        Fx=Fy=Fz=Tx=Ty=Tz=None
        if use_force:
            Fx, Fy, Fz, Tx, Ty, Tz = self._parse_force(get_force_fn, robot_pose)

        Ft = math.hypot(Fx, Fy) if (Fx is not None and Fy is not None) else None
        M  = math.sqrt(Tx*Tx + Ty*Ty + Tz*Tz) if (Tx is not None and Ty is not None and Tz is not None) else None

        # Learn slow baselines when pressure is low (free-space/cable only)
        pressure_low = (pressure is None) or (pressure <= self.p_off)
        if pressure_low:
            if Ft is not None: self.Ft_base.update(Ft)
            if M  is not None: self.M_base.update(M)

        Ft_base = self.Ft_base.y or 0.0
        M_base  = self.M_base.y  or 0.0
        Ft_dev  = (Ft - Ft_base) if Ft is not None else None
        M_dev   = (M  - M_base)  if M  is not None else None

        # Ratios (diagnostics)
        shear_ratio  = (Ft / abs(Fz)) if (Ft is not None and Fz is not None and abs(Fz) > 1e-6) else None
        moment_ratio = (M  / abs(Fz)) if (M  is not None and Fz is not None and abs(Fz) > 1e-6) else None

        # Proximity gate: only check F/T if we're somewhat near contact (optional)
        proximity_ok = (pressure is not None) and (pressure >= self.proximity_pressure_low) if self.proximity_pressure_low > 0 else True

        # Instantaneous big-contact condition (before persistence/dwell/cooldown)
        big_contact_inst = False
        if use_force and proximity_ok:
            comp = (Fz is not None) and (Fz < -self.fz_big_contact)
            shear_bad  = (shear_ratio  is not None) and (shear_ratio  > self.shear_ratio_threshold)
            moment_bad = (moment_ratio is not None) and (moment_ratio > self.moment_ratio_threshold)

            # Deviations must exceed deadbands to avoid reacting to cable drift
            dev_ok = ((Ft_dev is not None and Ft_dev > self.ft_deadband) or
                      (M_dev  is not None and M_dev  > self.m_deadband))

            big_contact_inst = comp or ((shear_bad or moment_bad) and dev_ok)

        # Persistence & dwell
        if big_contact_inst:
            self.big_contact_inst_frames += 1
            if self.big_contact_inst_frames == 1:
                self.last_big_contact_start_ts = ts
        else:
            self.big_contact_inst_frames = max(0, self.big_contact_inst_frames - 1)
            if self.big_contact_inst_frames == 0:
                self.last_big_contact_start_ts = None

        dwell_ok = (self.last_big_contact_start_ts is not None) and ((ts - self.last_big_contact_start_ts) >= self.dwell_time_s)
        persistence_ok = (self.big_contact_inst_frames >= self.persistence_frames)
        cooldown_ok = ((ts - self.last_trigger_ts) >= self.cooldown_s)

        big_contact = bool(persistence_ok and dwell_ok and cooldown_ok)

        # Hard safety (limits), with persistence and cooldown too
        over_force_inst = False
        if use_force:
            if Fz is not None and abs(Fz) > self.fz_hard_limit: over_force_inst = True
            if Ft is not None and Ft > self.ft_hard_limit:      over_force_inst = True
            if M  is not None and M  > self.m_hard_limit:       over_force_inst = True

        if over_force_inst:
            self.over_force_inst_frames += 1
        else:
            self.over_force_inst_frames = max(0, self.over_force_inst_frames - 1)

        over_force = (self.over_force_inst_frames >= self.persistence_frames) and cooldown_ok

        # If either triggers, start cooldown
        if big_contact or over_force:
            self.last_trigger_ts = ts

        # Controller feedback: pressure only
        feedback = pressure

        # Combined flags for convenience
        contact = self.contact_state
        safety_active = bool(big_contact or over_force)
        safety_reason = "none"
        if over_force:
            safety_reason = "over_force"
        elif big_contact:
            safety_reason = "big_contact"

        diag = {
            "ts": ts,
            "pressure": pressure,
            "contact_pressure": self.contact_state,

            # Wrench values (tool frame)
            "F_tool": {"Fx": Fx, "Fy": Fy, "Fz": Fz},
            "T_tool": {"Tx": Tx, "Ty": Ty, "Tz": Tz},

            # Magnitudes and baselines
            "Ft": Ft, "M": M,
            "Ft_baseline": Ft_base, "M_baseline": M_base,
            "Ft_dev": Ft_dev, "M_dev": M_dev,

            # Ratios (diagnostics)
            "shear_ratio": shear_ratio, "moment_ratio": moment_ratio,

            # Instantaneous and persisted safety checks
            "big_contact_inst": big_contact_inst,
            "big_contact_frames": self.big_contact_inst_frames,
            "big_contact": big_contact,
            "over_force_inst": over_force_inst,
            "over_force_frames": self.over_force_inst_frames,
            "over_force": over_force,

            # Combined contact/safety view
            "contact": contact,                 # same as contact_pressure
            "safety_active": safety_active,     # big_contact or over_force
            "safety_reason": safety_reason,     # "none" | "big_contact" | "over_force"
            "contact_flags": {                  # grouped for convenient logging/UI
                "contact": contact,
                "big_contact": big_contact,
                "over_force": over_force,
                "safety_active": safety_active,
                "safety_reason": safety_reason,
            },
        }
        return feedback, diag

    # Helpers

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
        Rz = np.array([[cz, -sz, 0],[sz, cz, 0],[0,0,1]])
        Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
        Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
        R = Rz @ Ry @ Rx

        F_b = np.array([Fx_b, Fy_b, Fz_b])
        T_b = np.array([Tx_b, Ty_b, Tz_b])
        F_t = R.T @ F_b
        T_t = R.T @ T_b
        return float(F_t[0]), float(F_t[1]), float(F_t[2]), float(T_t[0]), float(T_t[1]), float(T_t[2])
