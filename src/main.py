"""Drive a Universal Robots arm (UR3e/UR5e/...) with an Xbox 360 controller.

Run on Windows:  python main.py --ip 192.168.3.3 [--dry-run]

Controls:
  Left stick     TCP X / Y (BASE frame)
  RT / LT        TCP Z up / down (BASE frame)
  Right stick    tilt tool about its own X (up/down) and Y (left/right), about the TCP
  RB / LB        rotate tool CW / CCW about its own Z, about the TCP
  B (hold)       fine (slow) speed
  X              toggle gripper open/close (Robotiq Hand-E)
  Y (HOLD)       align tool straight down (tool Z = -base Z), keep heading; release = stop
  Start          clear error (close popups, unlock protective stop, revive RTDE script)
  Back           stop and exit
"""
import argparse
import time

from ur_control.geometry import angle_between, tool_down_rotvec, tool_z_axis
from ur_control.gripper import Gripper
from ur_control.jog import JogConfig, VelocityShaper, apply_limits, compute_twist, tilt_deg
from ur_control.recovery import clear_fault, fault_state
from xbox.xbox_control import BTN_BACK, BTN_START, BTN_X, BTN_Y, XboxController

LOOP_HZ = 100


def align_tool_down(rtde_r, rtde_c, joy, cfg, dry_run):
    pose = rtde_r.getActualTCPPose()
    err = angle_between(tool_z_axis(pose[3:]), [0, 0, -1])
    if err < 0.003:
        print("\n[align] already pointing down")
        return
    target = pose[:3] + tool_down_rotvec(pose[3:])
    print(f"\n[align] rotating tool down ({err * 57.296:.1f} deg off). Keep holding Y; release to stop.")
    if dry_run:
        return
    try:
        rtde_c.moveL(target, cfg.align_speed, cfg.align_accel, True)   # asynchronous
    except Exception as e:
        print(f"[align] move rejected: {e}")
        return
    while rtde_c.getAsyncOperationProgress() >= 0:
        p = joy.read()
        if not p.connected or p.down(BTN_BACK) or not p.down(BTN_Y):
            rtde_c.stopL(cfg.track_accel)
            print("[align] stopped (Y released)")
            return
        time.sleep(0.01)
    print("[align] done")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.3.3")
    ap.add_argument("--dry-run", action="store_true",
                    help="read robot pose + gamepad, print commands, never send motion")
    args = ap.parse_args()

    cfg = JogConfig()
    joy = XboxController()
    if not joy.read().connected:
        print("WARNING: no controller detected yet.")

    from rtde_receive import RTDEReceiveInterface
    rtde_r = RTDEReceiveInterface(args.ip)
    rtde_c = None
    if not args.dry_run:
        from rtde_control import RTDEControlInterface
        if rtde_r.getRobotMode() != 7:
            raise SystemExit(f"Robot not RUNNING (mode {rtde_r.getRobotMode()}). Power on + release brakes on the pendant.")
        if rtde_r.getSafetyMode() != 1:
            raise SystemExit(f"Safety mode {rtde_r.getSafetyMode()} (not NORMAL). Clear protective stop first.")
        # Requires the pendant to be in Remote Control mode.
        rtde_c = RTDEControlInterface(args.ip)

    gripper = Gripper(args.ip, rtde_c=rtde_c, dry_run=args.dry_run)
    origin = rtde_r.getActualTCPPose()
    print(f"Connected to {args.ip}. Start pose: {[round(v, 3) for v in origin]}")
    print(f"Workspace (base): X {cfg.x_min}..{cfg.x_max} m, Y {cfg.y_min}..{cfg.y_max} m, "
          f"Z +/-{cfg.box_z} m around start; tool tilt <= {cfg.max_tilt_deg} deg from vertical")
    if not (cfg.x_min <= origin[0] <= cfg.x_max and cfg.y_min <= origin[1] <= cfg.y_max):
        print("WARNING: start pose is OUTSIDE the XY workspace; only motion back inside is allowed.")
    if tilt_deg(origin) > cfg.max_tilt_deg:
        print(f"WARNING: tool is {tilt_deg(origin):.1f} deg from vertical; only tilting back is allowed (Y aligns).")
    print("NO ENABLE BUTTON: sticks/triggers/bumpers move the robot immediately. Back exits."
          + ("  [DRY RUN]" if args.dry_run else ""))

    dt = 1.0 / LOOP_HZ
    shaper = VelocityShaper(cfg.accel)
    t_prev = time.perf_counter()
    was_moving = False
    prev = 0
    announced = None
    motion_failed = False
    try:
        while True:
            t0 = time.perf_counter()
            dt_real = min(t0 - t_prev, 0.05)
            t_prev = t0
            pad = joy.read()
            if pad.down(BTN_BACK):
                break
            pressed = pad.buttons & ~prev          # rising edges
            prev = pad.buttons

            if pressed & BTN_X:
                gripper.toggle()
            if pressed & BTN_START:
                if args.dry_run:
                    print("\n[clear] (dry run)")
                else:
                    clear_fault(args.ip, rtde_r, rtde_c)
                announced = None
                motion_failed = False
            fault = fault_state(rtde_r, rtde_c) or ("motion command failed" if motion_failed else None)
            if fault:
                if fault != announced:
                    print(f"\nERROR: {fault}. Press Start to clear.")
                    announced = fault
                was_moving = False
                shaper.reset()
                time.sleep(dt)
                continue
            announced = None
            if pressed & BTN_Y:
                align_tool_down(rtde_r, rtde_c, joy, cfg, args.dry_run)
                was_moving = False
                shaper.reset()
                t_prev = time.perf_counter()
                prev = joy.read().buttons          # don't re-trigger on held buttons

            pose = rtde_r.getActualTCPPose()
            desired = apply_limits(compute_twist(pad, cfg, pose), pose, shaper.v, origin, cfg)
            twist = shaper.step(desired, dt_real)
            moving = any(twist)
            try:
                if args.dry_run:
                    if moving:
                        print("cmd", [round(v, 3) for v in twist], end="\r")
                elif moving:
                    rtde_c.speedL(twist, cfg.track_accel, cfg.cmd_timeout)
                elif was_moving:
                    rtde_c.speedStop(cfg.track_accel)
            except Exception as e:
                print(f"\nERROR: motion command failed: {e}")
                motion_failed = True
                moving = False
            was_moving = moving
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if rtde_c:
            rtde_c.speedStop(1.0)
            rtde_c.stopScript()
        print("\nStopped.")


if __name__ == "__main__":
    main()
