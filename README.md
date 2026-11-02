# UR Xbox Controller

Jog a Universal Robots arm (UR3e/UR5e/...) with an Xbox 360 controller over RTDE (`speedL`).

**Runs on native Windows Python** (XInput). WSL2's stock kernel has no joystick/xpad support.

## Setup (Windows)
1. Install Python 3.10+ from python.org (the Store stub is not enough).
2. Install the Xbox 360 Wireless Receiver driver (Windows Update / Microsoft Update Catalog); confirm with `joy.cpl`.
3. `pip install -r requirements.txt`
4. Pendant: power on, release brakes, switch to **Remote Control** (e-Series).
5. PC on the robot's subnet (robot: 192.168.3.3).

## Run
```
cd src
python xbox/xbox_control.py          # check controller input
python main.py --ip 192.168.3.3 --dry-run   # reads robot pose, prints commands, no motion
python main.py --ip 192.168.3.3      # live
```

## Controls (no enable button -- inputs act immediately)
| Input | Action |
|---|---|
| Left stick | TCP X / Y (**base** frame) |
| RT / LT | TCP Z up / down (**base** frame) |
| Right stick | Tilt tool about its own X (up/down) and Y (left/right), about the TCP |
| RB / LB | Rotate tool CW / CCW about its own Z axis, about the TCP |
| B (hold) | Fine speed (0.03 m/s vs 0.12 m/s) |
| X | Toggle gripper open/close (Robotiq Hand-E via URCap socket, port 63352) |
| Y (hold) | Align tool straight down (tool Z = -base Z), keeping heading; releasing Y stops the move |
| Start | Clear error: close popups, unlock protective stop, revive the RTDE script (press again if a protective stop is < 5 s old) |
| Back | Stop and exit |

## Gripper test
`python gripper_test.py` (from `src`) toggles the gripper twice and pops the gripper's replies on the pendant. Keep fingers clear.

## Safety behavior
- Workspace (base frame): X -0.400..-0.100 m, Y -0.300..-0.100 m, Z +/-0.30 m around the start pose, 10 mm stand-off.
  If the arm starts outside, only motion back inside is allowed.
- Tool Z is held within 30 deg of straight down (about 27 deg reachable: 3 deg margin, and tilt speed tapers earlier the faster you go). Tilt speed tapers to zero
  near the limit; spin about tool Z (bumpers) is unrestricted. Y re-aligns to vertical.
- Velocity is shaped in software at 0.8 m/s^2 per axis, so stopping distance is known and the limits above hold
  (verified in a closed-loop simulation with random inputs). `track_accel` (3.0) is how tightly the robot
  follows, and how fast it stops if commands stop arriving (0.1 s timeout). Controller disconnect = zero velocity.
- Faults (protective stop, safety mode, dead RTDE script, failed command) halt motion and print `ERROR ... Press Start`.
  Start never releases brakes or restarts safety: emergency stops need a physical reset on the pendant.
- Tune in `src/ur_control/jog.py` (`JogConfig`). Flip `sign_*` if directions don't match.
- Pendant speed slider and safety configuration still apply.
