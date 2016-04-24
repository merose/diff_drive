from __future__ import division
PKG='test_goal_controller'

import unittest
from math import pi, sin, cos
from diff_drive.goal_controller import GoalController
from diff_drive.pose import Pose


class TestGoalController(unittest.TestCase):

    def setUp(self):
        self.controller = GoalController()

    def testAtGoal(self):
        cur = Pose()
        desired = self.controller.getVelocity(cur, cur, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertEquals(desired.thetaVel, 0)

    def testStraightAhead(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertEquals(desired.thetaVel, 0)

    def testRotateLeft(self):
        cur = Pose()
        goal = Pose()
        goal.theta = pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    def testRotateRight(self):
        cur = Pose()
        goal = Pose()
        goal.theta = -pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    def testCurveLeft(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = 1
        goal.theta = pi
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    def testCurveRight(self):
        cur = Pose()
        cur.theta = pi
        goal = Pose()
        goal.x = 1
        goal.y = 1
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    def testButtonHookLeft(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.theta = pi
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    def testButtonHookRight(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.theta = -pi
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    def testGoToGoal(self):
        self.checkGoToGoal(0, 0, 0, 1, 1, pi/2)
        self.checkGoToGoal(0, 0, 0, 1, 1, -pi/2)
        self.checkGoToGoal(0, 0, 0, 0, 1, pi)
        self.checkGoToGoal(0, 0, 0, 1, 0, pi)

    def checkGoToGoal(self, x0, y0, th0, x1, y1, th1):
        dTol = 0.05 # 5cm
        thTol = 0.04 # Approx 2.5 degrees

        self.controller.setLinearTolerance(dTol)
        self.controller.setAngularTolerance(thTol)

        cur = Pose()
        cur.x = x0
        cur.y = y0
        cur.theta = th0

        goal = Pose()
        goal.x = x1
        goal.y = y1
        goal.theta = th1

        lastDistance = self.controller.getGoalDistance(cur, goal)
        dT = 0.05
        for i in range(1000):
            if self.controller.atGoal(cur, goal):
                return

            desired = self.controller.getVelocity(cur, goal, dT)
            cur.x += dT * desired.xVel * cos(cur.theta)
            cur.y += dT * desired.xVel * sin(cur.theta)
            cur.theta += dT * desired.thetaVel

        # If we get here, we didn't reach the goal.
        self.assertFalse('Did not reach the goal.')
        
if __name__ == '__main__':
    unittest.main()
