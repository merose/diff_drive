from __future__ import division
PKG='test_controller'

import unittest
from diff_drive.mock_robot import MockRobot


class TestMockRobot(unittest.TestCase):

    def setUp(self):
        self.robot = MockRobot()

    def testNoMotion(self):
        self.robot.updateRobot(1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, 0)
        self.assertEquals(ticks.right, 0)

        self.robot.updateRobot(1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, 0)
        self.assertEquals(ticks.right, 0)

    def testStraightLine(self):
        self.robot.setSpeeds(100, 100)
        self.robot.updateRobot(1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, 0)
        self.assertEquals(ticks.right, 0)

        self.robot.updateRobot(1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, 100)
        self.assertEquals(ticks.right, 100)

        self.robot.updateRobot(0.1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, 110)
        self.assertEquals(ticks.right, 110)

    def testRotateLeft(self):
        self.robot.setSpeeds(-100, 100)
        self.robot.updateRobot(0.1)

        self.robot.updateRobot(0.1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, -10)
        self.assertEquals(ticks.right, 10)

        self.robot.updateRobot(0.1)
        ticks = self.robot.getTicks()
        self.assertEquals(ticks.left, -20)
        self.assertEquals(ticks.right, 20)

        
if __name__ == '__main__':
    unittest.main()
