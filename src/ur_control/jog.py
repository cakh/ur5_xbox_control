"""Pure mapping from gamepad state to a TCP velocity command (no hardware deps)."""
from dataclasses import dataclass

import math

from ur_control.geometry import tool_to_base, tool_z_axis
from xbox.xbox_control import BTN_B, BTN_LB, BTN_RB


@dataclass
class JogConfig:
    speed: float = 0.12          # m/s at full deflection
    fine_speed: float = 0.03     # m/s while B held (precision mode)
    rot_speed: float = 0.45      # rad/s tool rotation (bumpers, right stick)
    deadzone: float = 0.15
    accel: float = 0.80          # m/s^2 and rad/s^2: our own per-axis ramp (shaper) + braking limits
    cmd_timeout: float = 0.1     # s; robot stops itself if no new command arrives
    # Absolute workspace in the BASE frame (meters). Z is relative to the start pose.
    x_min: float = -0.400
    x_max: float = -0.100        # keep away from the base singularity
    y_min: float = -0.300
    y_max: float = -0.100
    box_z: float = 0.30          # m, +/- from start pose Z
    max_tilt_deg: float = 30.0   # tool Z may be at most this far from straight down
    margin_xy: float = 0.010     # m, extra stand-off inside each workspace limit (tracking lag)
    margin_tilt_deg: float = 3.0 # deg, extra stand-off inside the tilt limit
    tilt_brake_scale: float = 2.0  # >1 = assume weaker tilt braking (tapers tilt speed earlier)
    track_accel: float = 3.0     # speedL accel param: robot tracks our shaped command closely.
                                 # (Also how fast it stops if commands stop arriving.)
    align_speed: float = 0.25    # moveL speed for the Y-button align move
    align_accel: float = 0.25
    # Base-frame mapping. Flip signs if stick direction doesn't match your robot orientation.
    sign_x: float = -1.0         # stick up -> robot -X (operator standing at UR base-frame front)
    sign_y: float = 1.0
    # Tool-frame rotations (about the TCP). Bumpers: tool Z, RB = CW looking along tool Z.
    sign_rot: float = 1.0
    # Right stick: Y -> rotation about tool X, X -> rotation about tool Y.
    sign_tool_x: float = 1.0
    sign_tool_y: float = 1.0


def shape(v, dz):
    """Deadzone with rescale so output ramps from 0 at the edge of the deadzone."""
    if abs(v) < dz:
        return 0.0
    s = (abs(v) - dz) / (1 - dz)
    return s if v > 0 else -s


def compute_twist(pad, cfg, pose):
    """Return [vx,vy,vz,wx,wy,wz] in base frame. `pose` is the actual TCP pose."""
    if not pad.connected:
        return [0.0] * 6
    spd = cfg.fine_speed if pad.down(BTN_B) else cfg.speed
    vx = cfg.sign_x * shape(pad.ly, cfg.deadzone) * spd
    vy = cfg.sign_y * shape(pad.lx, cfg.deadzone) * spd
    vz = (shape(pad.rt, cfg.deadzone) - shape(pad.lt, cfg.deadzone)) * spd
    # Angular velocity in the TOOL frame, then rotated into base axes: speedL angular part is
    # about the TCP, so with zero linear velocity the tool spins about its own TCP.
    w_tool = [cfg.sign_tool_x * shape(pad.ry, cfg.deadzone) * cfg.rot_speed,
              cfg.sign_tool_y * shape(pad.rx, cfg.deadzone) * cfg.rot_speed,
              cfg.sign_rot * cfg.rot_speed * (int(pad.down(BTN_RB)) - int(pad.down(BTN_LB)))]
    wb = tool_to_base(pose[3:], w_tool)
    return [vx, vy, vz, wb[0], wb[1], wb[2]]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _dot(a, b):
    return sum(p * q for p, q in zip(a, b))


def tilt_deg(pose):
    """Angle between tool Z and straight down, degrees."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -tool_z_axis(pose[3:])[2]))))


def limit_tilt(w, rv, w_actual, cfg):
    """Cap the tilt-plane speed of angular velocity `w` (base axes) by the room left before
    max_tilt_deg: s <= sqrt(2 * a * room), i.e. the speed from which we can still stop inside the
    limit. This also bleeds off sideways motion along the limit boundary (which otherwise drifts
    outward). Spin about tool Z and commands clearly heading back toward vertical are untouched."""
    z = tool_z_axis(rv)
    hn = math.hypot(z[0], z[1])
    if hn < 1e-9:
        return list(w)                              # exactly vertical: far from the limit
    u = [z[0] / hn, z[1] / hn, 0.0]                 # horizontal direction z is leaning toward
    a = _cross(z, u)
    an = math.sqrt(_dot(a, a))
    a = [c / an for c in a]                         # rotating about +a increases the tilt
    spin = _dot(w, z)
    wp = [w[i] - spin * z[i] for i in range(3)]     # tilt-plane part of the command
    s_cmd = math.sqrt(_dot(wp, wp))
    if s_cmd < 1e-9 or _dot(w, a) <= -0.25 * s_cmd:
        return list(w)                              # nothing to limit / clearly inward
    ap = [w_actual[i] - _dot(w_actual, z) * z[i] for i in range(3)]
    s_act = math.sqrt(_dot(ap, ap))
    theta = math.acos(max(-1.0, min(1.0, -z[2])))
    room = max(0.0, math.radians(cfg.max_tilt_deg - cfg.margin_tilt_deg) - theta)
    # Per-axis slewing decelerates the tilt motion at >= accel/sqrt(3) in the worst case.
    s_allowed = math.sqrt(2 * (cfg.accel / math.sqrt(3) / cfg.tilt_brake_scale) * room)
    scale = 0.0 if s_act > s_allowed else min(1.0, s_allowed / s_cmd)
    return [spin * z[i] + scale * wp[i] for i in range(3)]


def apply_limits(twist, pose, actual_speed, origin, cfg):
    """Zero velocity components that would carry the TCP out of the workspace box (accounting
    for braking distance v^2/(2*accel)) and limit tool tilt. Moving back toward the inside of
    a violated limit is always allowed."""
    out = list(twist)
    bounds = ((cfg.x_min, cfg.x_max), (cfg.y_min, cfg.y_max),
              (origin[2] - cfg.box_z, origin[2] + cfg.box_z))
    for i, (lo, hi) in enumerate(bounds):
        v = out[i]
        if v == 0.0:
            continue
        va = actual_speed[i] if actual_speed[i] * v > 0 else 0.0
        brake = max(abs(v), abs(va)) ** 2 / (2 * cfg.accel)
        m = cfg.margin_xy
        if (v > 0 and pose[i] + brake >= hi - m) or (v < 0 and pose[i] - brake <= lo + m):
            out[i] = 0.0
    out[3:] = limit_tilt(out[3:], pose[3:], actual_speed[3:], cfg)
    return out


class VelocityShaper:
    """Per-axis slew limiter: commanded velocity changes by at most `accel * dt` on each axis, so
    stopping distances are exactly v^2/(2*accel) per axis and the limits above hold."""

    def __init__(self, accel):
        self.accel = accel
        self.v = [0.0] * 6

    def step(self, desired, dt):
        step = self.accel * dt
        self.v = [c + max(-step, min(step, d - c)) for c, d in zip(self.v, desired)]
        return list(self.v)

    def reset(self):
        self.v = [0.0] * 6
