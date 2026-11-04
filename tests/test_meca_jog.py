import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from meca_control.jog import MecaJogConfig, Shaper, apply_box, compute_desired  # noqa: E402
from xbox.xbox_control import BTN_B, BTN_RB, Pad  # noqa: E402

CFG = MecaJogConfig()


class Jog(unittest.TestCase):
    def test_disconnected_is_zero(self):
        self.assertEqual(compute_desired(Pad(connected=False, lx=1, ly=1), CFG), ([0] * 3, [0] * 3))

    def test_deadzone(self):
        lin, ang = compute_desired(Pad(connected=True, lx=0.1, ly=-0.14), CFG)
        self.assertEqual(lin, [0, 0, 0])

    def test_full_deflection_and_fine(self):
        lin, _ = compute_desired(Pad(connected=True, ly=1.0), CFG)
        self.assertAlmostEqual(lin[0], CFG.speed)
        lin, _ = compute_desired(Pad(connected=True, ly=1.0, buttons=BTN_B), CFG)
        self.assertAlmostEqual(lin[0], CFG.fine_speed)

    def test_rotation_overrides_translation(self):
        lin, ang = compute_desired(Pad(connected=True, ly=1.0, buttons=BTN_RB), CFG)
        self.assertEqual(lin, [0, 0, 0])
        self.assertAlmostEqual(ang[2], CFG.rot_speed)

    def test_box_blocks_outward_allows_inward(self):
        box = [-100, -100, -100, 100, 100, 100]
        self.assertEqual(apply_box([50, 0, 0], [99, 0, 0], [0] * 3, box, CFG)[0], 0.0)
        self.assertEqual(apply_box([-50, 0, 0], [99, 0, 0], [0] * 3, box, CFG)[0], -50)
        # outside the box: only the way back in is allowed
        self.assertEqual(apply_box([50, 0, 0], [150, 0, 0], [0] * 3, box, CFG)[0], 0.0)
        self.assertEqual(apply_box([-50, 0, 0], [150, 0, 0], [0] * 3, box, CFG)[0], -50)

    def test_shaper_ramps_and_never_exceeds_accel(self):
        s = Shaper(CFG)
        name, v = s.step([50, 0, 0], [0] * 3, 0.02)
        self.assertEqual(name, "MoveLinVelWrf")
        self.assertAlmostEqual(v[0], CFG.accel * 0.02)

    def test_mutual_exclusion_no_abrupt_drop(self):
        s = Shaper(CFG)
        for _ in range(50):
            s.step([50, 0, 0], [0] * 3, 0.02)
        prev = s.lin.v[0]
        # user now rotates: translation must decay, not vanish, and rotation must wait
        name, v = s.step([0] * 3, [0, 0, 30], 0.02)
        self.assertEqual(name, "MoveLinVelWrf")
        self.assertAlmostEqual(prev - v[0], CFG.accel * 0.02)
        for _ in range(100):
            name, v = s.step([0] * 3, [0, 0, 30], 0.02)
        self.assertEqual(name, "MoveLinVelTrf")
        self.assertEqual(v[:3], [0, 0, 0])

    def test_random_inputs_stay_in_box(self):
        rnd = random.Random(1)
        box = [-100, -100, -100, 100, 100, 100]
        pose, s = [0.0, 0.0, 0.0], Shaper(CFG)
        dt = 0.02
        for _ in range(20000):
            pad = Pad(True, rnd.uniform(-1, 1), rnd.uniform(-1, 1), 0, 0, rnd.random(), rnd.random(), 0)
            lin, ang = compute_desired(pad, CFG)
            lin = apply_box(lin, pose, s.actual(), box, CFG)
            name, v = s.step(lin, ang, dt)
            pose = [p + v[i] * dt for i, p in enumerate(pose)]
            self.assertTrue(all(-100 <= p <= 100 for p in pose), pose)


if __name__ == "__main__":
    unittest.main()
