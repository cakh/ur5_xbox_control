# Meca500 Xbox jog

Windows Python (XInput), same controller reader as the UR tool.

```
cd src
python meca_main.py --ip <ROBOT_IP> --dry-run   # prints commands, no motion
python meca_main.py --ip <ROBOT_IP>             # live
```

Startup: clears errors, activates, homes (prompts first -- homing moves joints; `--yes` skips), clears the queue and resumes if paused (a dropped connection leaves it paused).

Prereqs: `pip install -r requirements.txt` (installs Mecademic's official `mecademicpy`). Close MecaPortal first (one control client only). `--dry-run` connects in monitoring mode only, so it cannot send anything.

| Input | Action |
|---|---|
| Left stick | TCP X / Y (WRF) |
| RT / LT | TCP Z up / down (WRF) |
| Right stick | Tilt about tool X / Y (TCP) |
| RB / LB | Spin about tool Z |
| B (hold) | Fine speed |
| X | Toggle gripper (untested) |
| Start | ResetError + ResumeMotion |
| Back | Stop and exit |

Safety: 50 mm/s max (10 fine), 200 mm/s^2 slew, box default +/-100 mm around start pose
(`--workspace`), `SetVelTimeout(0.1)` stops the robot if commands stop, stale pose / any robot
error halts until Start. Rotation overrides translation (one velocity command per tick).
Tune/flip signs in `src/meca_control/jog.py`. Tests: `python -m unittest discover -s tests`.
