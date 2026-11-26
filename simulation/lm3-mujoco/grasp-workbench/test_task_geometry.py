import unittest

import numpy as np

from scene import OBJECTS, candidates, inside_tray, tray_boxes


class SceneTest(unittest.TestCase):
    def test_tray_has_open_center(self):
        boxes = tray_boxes([-.5, .2, 0.])
        self.assertEqual(len(boxes), 5)
        for center, size in boxes[1:]:
            self.assertFalse(abs(center[0]+.5) < size[0]/2 and abs(center[1]-.2) < size[1]/2)

    def test_cup_must_fit_and_be_upright(self):
        tray = OBJECTS['tray']
        center = [tray[0], tray[1], tray[2]+.011]
        self.assertTrue(inside_tray(center, [0, 0, 1], tray))
        self.assertFalse(inside_tray([center[0]+.09, *center[1:]], [0, 0, 1], tray))
        self.assertFalse(inside_tray(center, [0, 1, 0], tray))

    def test_candidates_follow_object_pose_and_approach_from_above(self):
        cup = np.array([-.46, -.24, .02])
        options = candidates(cup)
        self.assertGreater(len(options), 1)
        for flange, approach, quat in options:
            np.testing.assert_allclose(flange[:2], cup[:2])
            self.assertGreater(approach[2], flange[2])
            self.assertAlmostEqual(np.linalg.norm(quat), 1.)


if __name__ == '__main__':
    unittest.main()
