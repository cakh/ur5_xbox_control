# Xbox Controller Robot Jog

> **New to GitHub? Use the friendly step-by-step guide: [nickarmenta.github.io/robot_xbox_control](https://nickarmenta.github.io/robot_xbox_control/)**

Drive a robot arm with an Xbox 360 controller from a Windows PC.

| Robot | Script | Connection |
|---|---|---|
| Universal Robots (UR3e / UR5e / ...) | `src/main.py` | RTDE (`speedL`) |
| Mecademic Meca500 | `src/meca_main.py` | TCP text API |

Every robot has its own IP address. Examples below use `<ROBOT_IP>`: replace it with yours (UR pendant: **Settings > System > Network**; otherwise your cell documentation or integrator). Always pass `--ip`.

> **WARNING: there is no enable button.** Sticks, triggers and bumpers move the robot the instant you touch them.
> Keep the e-stop in reach, keep people clear of the arm, and start with `--dry-run`.

This guide assumes you know the robot (pendant, Remote Control, e-stop, IP setup) but are new to GitHub and VS Code projects.

---

## 1. Install the tools (one time)

| Tool | Get it | Notes |
|---|---|---|
| Python **3.11 or 3.12** | [python.org/downloads](https://www.python.org/downloads/windows/) | Tick **"Add python.exe to PATH"** on the first installer screen. Do not use 3.13+ (no `ur_rtde` Windows package yet). The Microsoft Store stub is not enough. |
| Git | [git-scm.com/download/win](https://git-scm.com/download/win) | Accept the defaults. |
| VS Code | [code.visualstudio.com](https://code.visualstudio.com/) | |
| VS Code **Python** extension | In VS Code: `Ctrl+Shift+X`, search "Python" (Microsoft) | |

## 2. Get the code

A "repo" is just a project folder that Git keeps in sync with GitHub.

1. Open VS Code. Press `Ctrl+Shift+P`, type **Git: Clone**, press Enter.
2. Paste the repo URL (the green **Code** button on the GitHub page copies it). Sign in to GitHub if asked.
3. Pick a folder to put it in (for example `C:\Users\<you>\source`), then click **Open** when VS Code offers.

No Git? On the GitHub page click **Code > Download ZIP**, extract it, then **File > Open Folder** in VS Code. You won't get updates this way.

## 3. Install the Python packages (one time)

Open a terminal in VS Code: **Terminal > New Terminal** (or `` Ctrl+` ``). It opens in the project folder. Paste these one at a time:

```powershell
py -3.12 -m venv .venv          # use -3.11 if that is what you installed
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

You should now see `(.venv)` at the start of the prompt. If VS Code asks "use this environment for the workspace?" click **Yes**.
Every time you open a new terminal, run `.venv\Scripts\Activate.ps1` again (VS Code usually does it for you).

## 4. Connect the controller

Plug in the Xbox 360 wireless receiver (or a wired controller), press the controller's Guide button, and check it in Windows: press `Win+R`, type `joy.cpl`, Enter. The controller should be listed, and the sticks should move in its Properties window. If the receiver shows as an unknown device, install **Xbox 360 Wireless Receiver for Windows** through Windows Update / Microsoft Update Catalog.

## 5. Prepare the robot

**Universal Robots**
1. Power on, release brakes, and switch the pendant to **Remote Control** (e-Series).
2. Give the PC a static IP on the robot's subnet: same first three numbers as the robot, a different last number, mask `255.255.255.0` (Settings > Network > Ethernet > IP assignment > Edit). Use the robot's IP from the pendant.
3. Check: `ping <ROBOT_IP>` in the terminal.

**Meca500**
1. Power on. PC on the robot's subnet (same first three numbers as the robot, different last number, mask `255.255.255.0`).
2. **Close MecaPortal** and anything else controlling the robot. Only one control connection is allowed.
3. Check: `ping <ROBOT_IP>`.

## 6. Run it (always in this order)

```powershell
cd src
python xbox/xbox_control.py                  # 1. controller test: move sticks, numbers should change
```

**UR**
```powershell
python main.py --ip <ROBOT_IP> --dry-run    # 2. connects, reads pose, prints commands, NO motion
python main.py --ip <ROBOT_IP>              # 3. live
```

**Meca500**
```powershell
python meca_main.py --ip <ROBOT_IP> --dry-run   # 2. prints commands, NO motion
python meca_main.py --ip <ROBOT_IP>             # 3. live
```

The Meca script clears errors, activates, and **homes** on start. Homing moves the joints, so it asks for confirmation first (`--yes` skips the prompt). It also clears the motion queue and resumes if a dropped connection left the robot paused.

**To stop:** press **Back** on the controller, or `Ctrl+C` in the terminal. In an emergency use the pendant e-stop. Closing the terminal also stops it.

## Controls

| Input | UR | Meca500 |
|---|---|---|
| Left stick | TCP X / Y (**base** frame) | TCP X / Y (world frame) |
| RT / LT | TCP Z up / down (base frame) | TCP Z up / down (world frame) |
| Right stick | Tilt tool about its own X (up/down) and Y (left/right), about the TCP | Tilt about tool X / Y (TCP) |
| RB / LB | Rotate tool CW / CCW about its own Z, about the TCP | Spin about tool Z |
| B (hold) | Fine speed (0.03 m/s vs 0.12 m/s) | Fine speed (10 vs 50 mm/s) |
| X | Toggle gripper (Robotiq Hand-E; needs the Robotiq URCap on the pendant, otherwise skip it) | Toggle gripper (untested) |
| Y (hold) | Align tool straight down (tool Z = -base Z), keeping heading; releasing Y stops the move | n/a |
| Start | Clear error: close popups, unlock protective stop, revive the RTDE script (press again if a protective stop is < 5 s old) | `ResetError` + `ResumeMotion` |
| Back | Stop and exit | Stop and exit |

Meca500: rotation overrides translation (one velocity command per tick).

**Gripper test (UR, only if the Robotiq URCap is installed):** from `src`, `python gripper_test.py <ROBOT_IP>` toggles the gripper twice and shows the gripper's replies on the pendant. Keep fingers clear.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `python` is not recognized, or opens the Microsoft Store | Reinstall Python with **Add to PATH** ticked, or turn off *Settings > Apps > Advanced app settings > App execution aliases* for python.exe. `py` works even without PATH. |
| `Activate.ps1 cannot be loaded because running scripts is disabled` | Run once: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, then activate again. |
| `pip install` fails on `ur_rtde` | Python 3.13+ has no Windows package. Install 3.11 or 3.12, delete the `.venv` folder, redo step 3. |
| `ModuleNotFoundError: rtde_control` | The venv isn't active (no `(.venv)` in the prompt), or step 3 was skipped. |
| `WARNING: no controller detected` or controller test shows nothing | Check `joy.cpl`. Press the Guide button, replace batteries, re-plug the receiver. Close other apps that may be using the controller. |
| Can't connect / timeout | `ping` the robot. Check the PC's static IP and subnet, Wi-Fi/VPN fighting the route. |
| Windows Firewall popup appears on first run | Tick **Private networks** and click **Allow access**. If it was dismissed or you lack admin rights, the connection will time out: ask IT to allow `python.exe` on private networks. |
| UR connects but RTDE errors | Pendant not in **Remote Control**, brakes not released, or another RTDE client is connected. |
| Meca refuses connection | MecaPortal (or another client) is still connected. Close it. |
| `ERROR ... Press Start` | Protective stop, safety mode or failed command. Fix the cause, then press **Start**. Start never releases brakes or restarts safety: an e-stop needs a physical reset on the pendant. |
| Dry run prints commands but the robot doesn't move | That's what `--dry-run` does. Run without it. |
| Robot won't move further in some direction | You hit the workspace box (see Safety). If the arm starts outside it, only motion back inside is allowed. |
| An axis moves the wrong way | Flip the matching `sign_*` value in `src/ur_control/jog.py` (UR) or `src/meca_control/jog.py` (Meca). |
| X button does nothing (UR) | The gripper needs the Robotiq URCap installed and the gripper activated on the pendant. Without the URCap there is no gripper control: ignore X. |
| Robot stops by itself | Commands stopped arriving (0.1 s timeout), the controller disconnected, or a fault occurred. By design. |

Still stuck? Copy the full terminal output (red text and all) and send it with what you were doing.

## Getting updates

In VS Code: click the **Source Control** icon (left bar), then **...** > **Pull**. Or in the terminal: `git pull`.
Do not edit files other than the tuning values noted below; if Git complains about local changes, ask before forcing anything.

## Safety behavior

**UR**
- Workspace (base frame): X -0.400..-0.100 m, Y -0.300..-0.100 m, Z +/-0.30 m around the start pose, 10 mm stand-off.
  If the arm starts outside, only motion back inside is allowed.
- Tool Z is held within 30 deg of straight down (about 27 deg reachable: 3 deg margin, and tilt speed tapers earlier the faster you go). Tilt speed tapers to zero
  near the limit; spin about tool Z (bumpers) is unrestricted. Y re-aligns to vertical.
- Velocity is shaped in software at 0.8 m/s^2 per axis, so stopping distance is known and the limits above hold
  (verified in a closed-loop simulation with random inputs). `track_accel` (3.0) is how tightly the robot
  follows, and how fast it stops if commands stop arriving (0.1 s timeout). Controller disconnect = zero velocity.
- Faults (protective stop, safety mode, dead RTDE script, failed command) halt motion and print `ERROR ... Press Start`.
- Pendant speed slider and safety configuration still apply.
- Tune in `src/ur_control/jog.py` (`JogConfig`).

**Meca500**
- 50 mm/s max (10 fine), 200 mm/s^2 slew. Default box +/-100 mm around the start pose; change with
  `--workspace XMIN YMIN ZMIN XMAX YMAX ZMAX`.
- `SetVelTimeout(0.1)` stops the robot if commands stop. Stale pose or any robot error halts until Start.
- Tune in `src/meca_control/jog.py`. Tests: `python -m unittest discover -s tests` (from the repo root).
