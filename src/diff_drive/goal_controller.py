from __future__ import division, print_function
from math import pi, sqrt, sin, cos, atan2
from diff_drive.pose import Pose
#import rospy

class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3
        self.kA = 8
        self.kB = -1.5
        self.maxLinearSpeed = 1E9
        self.maxAngularSpeed = 1E9
        self.maxLinearAcceleration = 1E9
        self.linearTolerance = 0.025 # 2.5cm
        self.angularTolerance = 3/180*pi # 3 degrees
        self.forwardMovementOnly = False

    def setConstants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def setMaxLinearSpeed(self, speed):
        self.maxLinearSpeed = speed

    def setMaxAngularSpeed(self, speed):
        self.maxAngularSpeed = speed

    def setMaxLinearAcceleration(self, accel):
        self.maxLinearAcceleration = accel

    def setLinearTolerance(self, tolerance):
        self.linearTolerance = tolerance

    def setAngularTolerance(self, tolerance):
        self.angularTolerance = tolerance

    def setForwardMovementOnly(self, forwardOnly):
        self.forwardMovementOnly = forwardOnly

    def getGoalDistance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX*diffX + diffY*diffY)

    def atGoal(self, cur, goal):
        if goal is None:
            return True
        d = self.getGoalDistance(cur, goal)
        dTh = abs(self.normalizePi(cur.theta - goal.theta))
        return d < self.linearTolerance and dTh < self.angularTolerance

    def getVelocity(self, cur, goal, dT):
        desired = Pose()

        goal_heading = atan2(goal.y - cur.y, goal.x - cur.x)
        a = -cur.theta + goal_heading

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta = self.normalizePi(cur.theta - goal.theta)
        b = -theta - a

        # rospy.loginfo('cur=%f goal=%f a=%f b=%f', cur.theta, goal_heading,
        #               a, b)

        d = self.getGoalDistance(cur, goal)
        if self.forwardMovementOnly:
            direction = 1
            a = self.normalizePi(a)
            b = self.normalizePi(b)
        else:
            direction = self.sign(cos(a))
            a = self.normalizeHalfPi(a)
            b = self.normalizeHalfPi(b)

        # rospy.loginfo('After normalization, a=%f b=%f', a, b)

        if abs(d) < self.linearTolerance:
            desired.xVel = 0
            desired.thetaVel = self.kB * theta
        else:
            desired.xVel = self.kP * d * direction
            desired.thetaVel = self.kA*a + self.kB*b

        # Adjust velocities if X velocity is too high.
        if abs(desired.xVel) > self.maxLinearSpeed:
            ratio = self.maxLinearSpeed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired.thetaVel) > self.maxAngularSpeed:
            ratio = self.maxAngularSpeed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        return desired

    def normalizeHalfPi(self, alpha):
        alpha = self.normalizePi(alpha)
        if alpha > pi/2:
            return alpha - pi
        elif alpha < -pi/2:
            return alpha + pi
        else:
            return alpha

    def normalizePi(self, alpha):
        if alpha > pi:
            return alpha - 2*pi
        elif alpha < -pi:
            return alpha + 2*pi
        else:
            return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
