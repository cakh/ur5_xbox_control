"""Jog a Mecademic Meca500 with an Xbox 360 controller (Windows, XInput).

  python meca_main.py --ip <ROBOT_IP> --dry-run     # read pose + pad, print commands, no motion
  python meca_main.py --ip <ROBOT_IP>               # live

Uses Mecademic's official `mecademicpy` library (pip install -r requirements.txt).

On start it clears errors, activates, homes (asks first: homing moves joints; --yes skips) and
resumes if paused. --dry-run connects in monitoring mode only (no control connection, nothing
can be sent). Only one client may hold the control connection: close MecaPortal first.

Controls (no enable button; the robot's own velocity timeout stops it if commands stop):
  Left stick     TCP X / Y (work frame)        RT / LT   TCP Z up / down
  Right stick    tilt about tool X / Y (TCP)   RB / LB   spin about tool Z
  B (hold)       fine speed                    X         toggle gripper (untested)
  Start          ResetError + ResumeMotion     Back      stop and exit
Rotation overrides translation (Meca velocity mode accepts one command per tick).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mecademicpy.robot as mdr

from meca_control.jog import MecaJogConfig, Shaper, apply_box, compute_desired
from xbox.xbox_control import BTN_BACK, BTN_START, BTN_X, XboxController

LOOP_HZ = 50
POSE_MAX_AGE = 0.25   # s without a new robot-stamped pose -> stop


class PoseFeed:
    """Latest TCP pose (WRF mm/deg) from the robot's real-time feed, with a staleness check."""

    def __init__(self, robot):
        self.robot = robot
        self.stamp = None
        self.seen = time.monotonic()
        self.pose = None

    def update(self):
        t = self.robot.GetRtTargetCartPos(include_timestamp=True, synchronous_update=False)
        if t.timestamp != self.stamp:
            self.stamp, self.seen = t.timestamp, time.monotonic()
        self.pose = list(t.data)

    def age(self):
        return time.monotonic() - self.seen


def fault(status):
    """Text describing why the robot can't move right now, or '' if it can."""
    if status.error_status:
        return f"[{status.error_code}] {status.error_msg}"
    if status.pause_motion_status:
        return "motion paused (safety stop or lost connection)"
    return ""


def bring_up(robot, args):
    """Get the robot ready for motion: ResetError, ActivateRobot, Home, then clear any stale
    queue and resume (a dropped control connection leaves the robot paused). Home moves joints,
    so it asks first unless --yes. --dry-run only reports."""
    st = robot.GetStatusRobot(synchronous_update=not args.dry_run)
    todo = []
    if st.error_status:
        todo.append("ResetError")
    if not st.activation_state:
        todo.append("ActivateRobot")
    if not st.homing_state:
        todo.append("Home (MOVES JOINTS)")
    if st.pause_motion_status or st.error_status:
        todo.append("ClearMotion + ResumeMotion")
    if not todo:
        return
    print("Robot needs:", ", ".join(todo))
    if args.dry_run:
        raise SystemExit(f"Dry run: not touching the robot. Status: {st}")
    if "Home (MOVES JOINTS)" in todo and not args.yes:
        if input("Homing moves the joints. Clear the cell and press Enter to continue (Ctrl+C aborts): ") != "":
            raise SystemExit("Aborted.")
    if st.error_status:
        robot.ResetError()
        time.sleep(0.3)
    if not st.activation_state:
        robot.ActivateRobot()
        robot.WaitActivated(timeout=30)
    if not st.homing_state:
        robot.Home()
        robot.WaitHomed(timeout=90)
    st = robot.GetStatusRobot(synchronous_update=True)
    if st.pause_motion_status:
        robot.ClearMotion()
        robot.ResumeMotion()
        time.sleep(0.3)
        st = robot.GetStatusRobot(synchronous_update=True)
    if not (st.activation_state and st.homing_state) or st.error_status or st.pause_motion_status:
        raise SystemExit(f"Robot still not ready for motion: {st}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.0.100", help="robot IP address (always pass yours)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation before homing")
    ap.add_argument("--workspace", type=float, nargs=6, metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
                    help="WRF box in mm (default: +/-100 mm around the start pose)")
    args = ap.parse_args()

    cfg = MecaJogConfig()
    joy = XboxController()
    if not joy.read().connected:
        print("WARNING: no controller detected yet.")

    robot = mdr.Robot()
    robot.Connect(address=args.ip, monitor_mode=args.dry_run, timeout=5.0)
    try:
        if args.dry_run:
            time.sleep(0.5)   # monitoring mode: wait for the first status/pose frames
        bring_up(robot, args)
        feed = PoseFeed(robot)
        deadline = time.time() + 2
        while time.time() < deadline:
            try:
                feed.update()
                if feed.pose and feed.stamp:
                    break
            except Exception:
                pass
            time.sleep(0.05)
        if not feed.pose:
            raise SystemExit("No pose received from the robot.")
        origin = list(feed.pose)
        box = args.workspace or [origin[0] - 100, origin[1] - 100, origin[2] - 100,
                                 origin[0] + 100, origin[1] + 100, origin[2] + 100]
        print(f"Connected to {args.ip}. Start pose: {[round(v, 2) for v in origin]}")
        print(f"Workspace (WRF mm): {[round(v, 1) for v in box]}")
        if not args.dry_run:
            robot.SetVelTimeout(cfg.vel_timeout)
        print("Running. Back to exit." + (" [DRY RUN]" if args.dry_run else ""))
        run(robot, feed, joy, cfg, box, args)
    finally:
        try:
            if not args.dry_run and robot.IsConnected():
                robot.MoveLinVelWrf(0, 0, 0, 0, 0, 0)
                time.sleep(0.1)
        except Exception:
            pass
        try:
            robot.Disconnect()
        except Exception:
            pass
        print("\nStopped.")


def run(robot, feed, joy, cfg, box, args):
    shaper = Shaper(cfg)
    gripper_open = None
    x_prev = False
    dt = 1.0 / LOOP_HZ
    last = time.monotonic()
    halted = ""
    while True:
        time.sleep(max(0.0, dt - (time.monotonic() - last)))
        now = time.monotonic()
        step, last = now - last, now
        pad = joy.read()
        if pad.down(BTN_BACK):
            return
        if not robot.IsConnected():
            raise SystemExit("\nERROR connection to robot lost.")

        reason = fault(robot.GetStatusRobot(synchronous_update=False))
        if reason and not halted:
            halted = reason
        if halted:
            shaper.reset()
            print(f"\rERROR {halted}. Press Start.        ", end="")
            if pad.down(BTN_START):
                if not args.dry_run:
                    robot.ResetError()
                    time.sleep(0.3)
                    robot.ResumeMotion()
                    time.sleep(0.3)
                halted = ""
            continue

        feed.update()
        if feed.age() > POSE_MAX_AGE:
            shaper.reset()
            continue
        pose = feed.pose

        x = pad.down(BTN_X)
        if x and not x_prev and not args.dry_run:
            gripper_open = not gripper_open if gripper_open is not None else True
            robot.GripperOpen() if gripper_open else robot.GripperClose()
        x_prev = x

        lin, ang = compute_desired(pad, cfg)
        lin = apply_box(lin, pose, shaper.actual(), box, cfg)
        name, vals = shaper.step(lin, ang, step)
        vals = [round(v, 3) for v in vals]
        if args.dry_run:
            if any(abs(v) > 1e-6 for v in vals):
                print(f"\r{name}({', '.join(str(v) for v in vals)})".ljust(70), end="")
        else:
            try:
                getattr(robot, name)(*vals)
            except Exception as e:   # the library disconnects on a failed command
                raise SystemExit(f"\nERROR motion command failed: {e}")


if __name__ == "__main__":
    main()
