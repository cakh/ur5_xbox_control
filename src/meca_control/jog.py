"""Pure mapping from gamepad state to a Meca500 velocity command (no hardware deps).

Meca velocity mode takes ONE command per tick, so translation and rotation are exclusive:
  * any rotation input  -> MoveLinVelTrf(0,0,0,wx,wy,wz)  (pure spin about the TCP, tool axes)
  * otherwise           -> MoveLinVelWrf(vx,vy,vz,0,0,0)  (translate in the work frame)
Units: mm/s and deg/s. Pose is [x,y,z,alpha,beta,gamma] in the WRF (mm, deg).
"""
from dataclasses import dataclass

from ur_control.jog import shape
from xbox.xbox_control import BTN_B, BTN_LB, BTN_RB


@dataclass
class MecaJogConfig:
    speed: float = 50.0          # mm/s at full deflection
    fine_speed: float = 10.0     # mm/s while B held
    rot_speed: float = 30.0      # deg/s
    fine_rot_speed: float = 8.0
    deadzone: float = 0.15
    accel: float = 200.0         # mm/s^2 our own slew limit (also sets the braking distance)
    rot_accel: float = 120.0     # deg/s^2
    margin: float = 3.0          # mm stand-off inside each workspace face
    vel_timeout: float = 0.1     # s; robot stops itself if commands stop (SetVelTimeout)
    # Flip if stick direction doesn't match how you stand relative to the robot (WRF).
    sign_x: float = 1.0
    sign_y: float = 1.0
    sign_z: float = 1.0
    sign_rot: tuple = (1.0, 1.0, 1.0)   # tool X / Y / Z spin


def compute_desired(pad, cfg):
    """-> (linear[3] mm/s WRF, angular[3] deg/s TRF). Rotation wins over translation."""
    if not pad.connected:
        return [0.0] * 3, [0.0] * 3
    fine = pad.down(BTN_B)
    w = cfg.fine_rot_speed if fine else cfg.rot_speed
    ang = [cfg.sign_rot[0] * shape(pad.ry, cfg.deadzone) * w,
           cfg.sign_rot[1] * shape(pad.rx, cfg.deadzone) * w,
           cfg.sign_rot[2] * w * (int(pad.down(BTN_RB)) - int(pad.down(BTN_LB)))]
    if any(ang):
        return [0.0] * 3, ang
    v = cfg.fine_speed if fine else cfg.speed
    lin = [cfg.sign_x * shape(pad.ly, cfg.deadzone) * v,
           cfg.sign_y * shape(pad.lx, cfg.deadzone) * v,
           cfg.sign_z * (shape(pad.rt, cfg.deadzone) - shape(pad.lt, cfg.deadzone)) * v]
    return lin, [0.0] * 3


def apply_box(lin, pose, actual_v, box, cfg):
    """Zero components that would carry the TCP out of `box` (xmin,ymin,zmin,xmax,ymax,zmax),
    allowing for braking distance v^2/(2a). Moving back inside a violated limit is allowed."""
    out = list(lin)
    for i in range(3):
        v = out[i]
        if v == 0.0:
            continue
        va = actual_v[i] if actual_v[i] * v > 0 else 0.0
        brake = max(abs(v), abs(va)) ** 2 / (2 * cfg.accel)
        lo, hi = box[i], box[i + 3]
        if (v > 0 and pose[i] + brake >= hi - cfg.margin) or (v < 0 and pose[i] - brake <= lo + cfg.margin):
            out[i] = 0.0
    return out


class Slew:
    """Per-axis slew limiter on the commanded velocity (so stopping distance is known)."""

    def __init__(self, accel):
        self.accel = accel
        self.v = [0.0] * 3

    def step(self, desired, dt):
        s = self.accel * dt
        self.v = [c + max(-s, min(s, d - c)) for c, d in zip(self.v, desired)]
        return list(self.v)

    def reset(self):
        self.v = [0.0] * 3


class Shaper:
    """Linear + angular slew with mutual exclusion: a new rotation waits until translation has
    decayed to zero (and vice versa), so the one command sent per tick never drops the other
    motion abruptly. step() returns (command_name, 6 values) for this tick."""

    EPS = 1e-6

    def __init__(self, cfg):
        self.lin, self.ang = Slew(cfg.accel), Slew(cfg.rot_accel)

    def step(self, lin, ang, dt):
        if any(abs(c) > self.EPS for c in self.lin.v):
            ang = [0.0] * 3
        if any(abs(c) > self.EPS for c in self.ang.v):
            lin = [0.0] * 3
        v, w = self.lin.step(lin, dt), self.ang.step(ang, dt)
        if any(abs(c) > self.EPS for c in w):
            return "MoveLinVelTrf", [0.0] * 3 + w
        return "MoveLinVelWrf", v + [0.0] * 3

    def actual(self):
        return list(self.lin.v)

    def reset(self):
        self.lin.reset()
        self.ang.reset()
