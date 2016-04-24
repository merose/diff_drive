from __future__ import division
PKG='test_controller'

import unittest
from diff_drive.controller import Controller


class TestController(unittest.TestCase):

    def setUp(self):
        self.ticksPerMeter = 10000
        self.wheelSeparation = 0.1
        self.controller = Controller()
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setWheelSeparation(self.wheelSeparation)

    def testStraightForward(self):
        speeds = self.controller.getSpeeds(0.1, 0)
        self.assertAlmostEqual(speeds.left, self.ticksPerMeter*0.1)
        self.assertAlmostEqual(speeds.right, self.ticksPerMeter*0.1)

    def testStraightBackward(self):
        speeds = self.controller.getSpeeds(-0.1, 0)
        self.assertAlmostEqual(speeds.left, -self.ticksPerMeter*0.1)
        self.assertAlmostEqual(speeds.right, -self.ticksPerMeter*0.1)

    def testRotateLeft(self):
        speeds = self.controller.getSpeeds(0, 1)
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        self.assertAlmostEqual(speeds.left, -diffTicks)
        self.assertAlmostEqual(speeds.right, diffTicks)

    def testRotateRight(self):
        speeds = self.controller.getSpeeds(0, -1)
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        self.assertAlmostEqual(speeds.left, diffTicks)
        self.assertAlmostEqual(speeds.right, -diffTicks)

    def testCurveLeft(self):
        speeds = self.controller.getSpeeds(0.1, 1)
        aheadTicks = 0.1 * self.ticksPerMeter
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        self.assertAlmostEqual(speeds.left, aheadTicks-diffTicks)
        self.assertAlmostEqual(speeds.right, aheadTicks+diffTicks)

    def testMotorLimitsStraight(self):
        maxTickSpeed = self.ticksPerMeter // 4
        self.controller.setMaxMotorSpeed(maxTickSpeed)
        speeds = self.controller.getSpeeds(1, 0)
        self.assertEqual(speeds.left, maxTickSpeed)
        self.assertEqual(speeds.right, maxTickSpeed)

    def testMotorLimitsCurved(self):
        maxTickSpeed = self.ticksPerMeter // 4
        self.controller.setMaxMotorSpeed(maxTickSpeed)
        speeds = self.controller.getSpeeds(1, 1)
        aheadTicks = self.ticksPerMeter
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        factor = maxTickSpeed / (aheadTicks + diffTicks)
        self.assertEqual(speeds.left,
                         int((aheadTicks-diffTicks) * factor))
        self.assertEqual(speeds.right, maxTickSpeed)

        
if __name__ == '__main__':
    unittest.main()
