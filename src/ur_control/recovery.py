"""Clear robot faults (popups, protective stop) via the dashboard server and revive the RTDE script."""
import socket
import time

SAFETY_NAMES = {1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
                6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP", 8: "VIOLATION", 9: "FAULT"}


def dashboard(ip, cmds, timeout=2.0):
    out = []
    with socket.create_connection((ip, 29999), timeout=timeout) as s:
        s.settimeout(timeout)
        s.recv(256)
        for c in cmds:
            s.sendall((c + "\n").encode())
            out.append((c, s.recv(256).decode().strip()))
    return out


def fault_state(rtde_r, rtde_c):
    """Return a description of what's wrong, or None if the robot is ready to be driven."""
    sm, rm = rtde_r.getSafetyMode(), rtde_r.getRobotMode()
    if sm not in (1, 2):
        return f"safety mode {sm} ({SAFETY_NAMES.get(sm, '?')})"
    if rm != 7:
        return f"robot mode {rm} (not RUNNING)"
    if rtde_c is not None and not rtde_c.isProgramRunning():
        return "RTDE control script not running"
    return None


def clear_fault(ip, rtde_r, rtde_c):
    """Close popups, unlock a protective stop, and re-upload the control script if it died.
    Never releases brakes or restarts safety: emergency stops need a physical reset."""
    sm = rtde_r.getSafetyMode()
    cmds = ["close popup", "close safety popup"]
    if sm == 3:
        cmds.append("unlock protective stop")
    try:
        for c, r in dashboard(ip, cmds):
            print(f"\n[clear] {c}: {r}", end="")
    except OSError as e:
        print(f"\n[clear] dashboard unreachable: {e}")
        return False
    time.sleep(1.0)
    problem = fault_state(rtde_r, None)
    if problem:
        hint = " (protective stop can only be unlocked 5 s after it happens; press X again)" if sm == 3 else \
               " (needs physical reset / pendant)" if sm in (5, 6, 7, 8, 9) else ""
        print(f"\n[clear] still faulted: {problem}{hint}")
        return False
    if rtde_c is not None and not rtde_c.isProgramRunning():
        for fn in ("reuploadScript", "reconnect"):
            try:
                getattr(rtde_c, fn)()
            except Exception as e:
                print(f"\n[clear] {fn} failed: {e}")
            time.sleep(0.5)
            if rtde_c.isProgramRunning():
                break
        if not rtde_c.isProgramRunning():
            print("\n[clear] control script still not running; restart main.py")
            return False
    print("\n[clear] OK, ready")
    return True
