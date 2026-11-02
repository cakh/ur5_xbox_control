"""Robotiq Hand-E over the Robotiq URCap socket server (robot IP, port 63352).

Requires the Robotiq Grippers URCap installed on the robot and the gripper activated.

Two paths, tried in this order:
  1. Direct TCP from the PC to <robot>:63352 (often firewalled on PolyScope 5.x).
  2. Relay: a URScript function run on the robot via RTDE (sendCustomScriptFunction) that
     talks to the URCap server on 127.0.0.1:63352 from inside the controller.
Protocol is line-based text: "SET POS 255\\n" -> "ack". POS 0 = fully open, 255 = fully closed.
A failure never raises into the driving loop; it just prints and retries on the next toggle.
"""
import socket

PORT = 63352


class Gripper:
    OPEN_POS = 0
    CLOSED_POS = 255

    def __init__(self, ip, rtde_c=None, speed=100, force=100, dry_run=False, debug=False):
        self.ip, self.speed, self.force, self.dry_run = ip, speed, force, dry_run
        self.debug = debug
        self.rtde_c = rtde_c
        self.closed = False
        self._sock = None
        if not dry_run:
            self._connect(quiet=True)
            pos = self._query("POS")
            if pos is not None:
                self.closed = pos > 127
                print(f"[gripper] direct socket OK, POS={pos} ({'closed' if self.closed else 'open'})")
            elif rtde_c is not None:
                print("[gripper] direct socket unavailable; using RTDE relay via robot "
                      "(assuming gripper starts OPEN)")
            else:
                print(f"[gripper] cannot reach {ip}:{PORT} and no RTDE relay available")

    def _connect(self, quiet=False):
        try:
            self._sock = socket.create_connection((self.ip, PORT), timeout=1.0)
            self._sock.settimeout(0.5)
        except OSError as e:
            self._sock = None
            if not quiet:
                print(f"\n[gripper] direct connect to {self.ip}:{PORT} failed ({e})")

    def _relay(self, pos):
        """Run the gripper commands as URScript on the robot, against the local URCap server."""
        cmds = [f"SET SPE {self.speed}", f"SET FOR {self.force}", f"SET POS {pos}", "SET GTO 1"]
        lines = ['rep = ""', 'if socket_open("127.0.0.1", 63352, "rq_grip"):']
        for c in cmds:
            lines.append(f'  socket_send_line("{c}", "rq_grip")')
            lines.append('  r = socket_read_string("rq_grip", "", "", False, 0.5)')
            lines.append('  rep = rep + r + " | "')
        lines += ['  socket_close("rq_grip")',
                  '  textmsg("xbox_gripper replies: ", rep)']
        if self.debug:
            lines.append('  popup(rep, "Gripper replies", False, False, False)')
        lines += ['else:',
                  '  popup("Robotiq socket 127.0.0.1:63352 unavailable", "Gripper", False, True, False)',
                  'end']
        try:
            ok = bool(self.rtde_c.sendCustomScriptFunction("xbox_gripper", "\n".join(lines)))
            print(f"\n[gripper] relay script sent, ok={ok}")
            return ok
        except Exception as e:
            print(f"\n[gripper] RTDE relay failed: {e}")
            return False

    def _txrx(self, cmd):
        self._sock.sendall((cmd + "\n").encode())
        return self._sock.recv(64).decode().strip()

    def _query(self, var):
        if self._sock is None:
            return None
        try:
            reply = self._txrx(f"GET {var}")      # e.g. "POS 123"
            return int(reply.split()[-1])
        except (OSError, ValueError):
            return None

    def _move(self, pos):
        if self.dry_run:
            print(f"\n[gripper] (dry run) would move to POS {pos}")
            return True
        if self._sock is None:
            self._connect(quiet=True)
        if self._sock is None:
            return self._relay(pos) if self.rtde_c is not None else False
        try:
            self._txrx(f"SET SPE {self.speed}")
            self._txrx(f"SET FOR {self.force}")
            self._txrx(f"SET POS {pos}")
            self._txrx("SET GTO 1")
            return True
        except OSError as e:
            print(f"\n[gripper] command failed: {e}")
            self._sock = None
            return False

    def toggle(self):
        target = self.OPEN_POS if self.closed else self.CLOSED_POS
        if self._move(target):
            self.closed = target == self.CLOSED_POS
            print(f"\n[gripper] {'CLOSE' if self.closed else 'OPEN'}")
        else:
            print("\n[gripper] toggle failed")
