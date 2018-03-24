from __future__ import division, print_function
PKG='test_goal_controller'

import unittest
from math import pi, sin, cos
from diff_drive.goal_controller import GoalController
from diff_drive.pose import Pose


class TestGoalController(unittest.TestCase):

    def setUp(self):
        self.controller = GoalController()

    # Test that the robot does not move when already at the goal
    # at the right heading.
    def testAtGoal(self):
        cur = Pose()
        desired = self.controller.getVelocity(cur, cur, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertEquals(desired.thetaVel, 0)

    # Test that a goal pose ahead of the current position at the same
    # heading causes a straight-ahead move.
    def testStraightAhead(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertEquals(desired.thetaVel, 0)

    # Test that a goal pose behind the current position at the same
    # heading causes a straight-back move.
    def testStraightBack(self):
        cur = Pose()
        goal = Pose()
        goal.x = -1
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertLess(desired.xVel, 0)
        self.assertEquals(desired.thetaVel, 0)

    # Test that a goal at the current position with a leftward goal
    # heading causes a leftward rotation.
    def testRotateLeft(self):
        cur = Pose()
        goal = Pose()
        goal.theta = pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    # Test that a goal at the current position with a rightward goal
    # heading causes a rightward rotation.
    def testRotateRight(self):
        cur = Pose()
        goal = Pose()
        goal.theta = -pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertEquals(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    # Test that a goal pose that is reachable with a forward, leftward
    # arc causes a forward movement with some leftward rotation.
    def testCurveLeft(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = 1
        goal.theta = pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    # Test that a goal pose that is reachable with a forward, rightward
    # arc causes a forward movement with some rightward rotation.
    def testCurveRight(self):
        cur = Pose()
        cur.theta = pi/2
        goal = Pose()
        goal.x = 1
        goal.y = 1
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    # Test that a goal pose behind the robot that is reachable with a
    # leftward arc causes a backward movement with some rightward
    # rotation.
    def testCurveBackLeft(self):
        cur = Pose()
        goal = Pose()
        cur.x = 1
        cur.y = 1
        goal.theta = pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertLess(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    # Test that a goal pose behind the robot that is reachable with a
    # rightward arc causes a backward movement with some leftward
    # rotation.
    def testCurveBackRigth(self):
        cur = Pose()
        goal = Pose()
        cur.x = 1
        cur.y = 1
        cur.theta = pi/2
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertLess(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    def testButtonHookLeft(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = 1
        goal.theta = pi
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertGreater(desired.thetaVel, 0)

    def testButtonHookRight(self):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = -1
        goal.theta = -pi
        desired = self.controller.getVelocity(cur, goal, 0.1)
        self.assertGreater(desired.xVel, 0)
        self.assertLess(desired.thetaVel, 0)

    def testGoToGoal(self):
        self.checkGoToGoal(0, 0, 0, 1, 0, 0) # Straight ahead
        self.checkGoToGoal(0, 0, 0, 1, 1, pi/2) # Arc left
        self.checkGoToGoal(0, 0, 0, 1, 1, -pi/2) # Left, then turn right
        self.checkGoToGoal(0, 0, 0, 0, 1, pi) # Go left
        self.checkGoToGoal(0, 0, 0, 1, 0, pi) # Go ahead and u-turn
        self.checkGoToGoal(0, 0, 0, -1, 0, 0) # Straight back
        self.checkGoToGoal(0, 0, 0, -1, -1, 0) # Back up to right
        self.checkGoToGoal(0, 0, 0, -1, -1, pi) # Back up and turn left

    def testGoToGoalForwardOnly(self):
        self.controller.setForwardMovementOnly(True)
        self.checkGoToGoal(0, 0, 0, 1, 0, 0) # Straight ahead
        self.checkGoToGoal(0, 0, 0, 1, 1, pi/2) # Arc left
        self.checkGoToGoal(0, 0, 0, 1, 1, -pi/2) # Left, then turn right
        self.checkGoToGoal(0, 0, 0, 0, 1, pi) # Go left
        self.checkGoToGoal(0, 0, 0, 1, 0, pi) # Go ahead and u-turn
        self.checkGoToGoal(0, 0, 0, -1, 0, 0) # Straight back
        self.checkGoToGoal(0, 0, 0, -1, -1, 0) # Back up to right
        self.checkGoToGoal(0, 0, 0, -1, -1, pi) # Back up and turn left

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
            newTheta = cur.theta + dT*desired.thetaVel
            midTheta = (cur.theta + newTheta) / 2.0
            cur.x += dT * desired.xVel * cos(midTheta)
            cur.y += dT * desired.xVel * sin(midTheta)
            cur.theta = newTheta

        # If we get here, we didn't reach the goal.
        self.assertFalse('Did not reach the goal: p0='
                         + str((x0,y0,th0))
                         + ' p1=' + str((x1,y1,th1)))
        
if __name__ == '__main__':
    unittest.main()
