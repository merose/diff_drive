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
        self.max_linear_speed = 1E9
        self.min_linear_speed = 0
        self.max_angular_speed = 1E9
        self.min_angular_speed = 0
        self.max_linear_acceleration = 1E9
        self.max_angular_acceleration = 1E9
        self.linear_tolerance = 0.025 # 2.5cm
        self.angular_tolerance = 3/180*pi # 3 degrees
        self.forward_movement_only = False

    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def set_max_linear_speed(self, speed):
        self.max_linear_speed = speed

    def set_min_linear_speed(self, speed):
        self.min_linear_speed = speed

    def set_max_angular_speed(self, speed):
        self.max_angular_speed = speed

    def set_min_angular_speed(self, speed):
        self.min_angular_speed = speed

    def set_max_linear_acceleration(self, accel):
        self.max_linear_acceleration = accel

    def set_max_angular_acceleration(self, accel):
        self.max_angular_acceleration = accel

    def set_linear_tolerance(self, tolerance):
        self.linear_tolerance = tolerance

    def set_angular_tolerance(self, tolerance):
        self.angular_tolerance = tolerance

    def set_forward_movement_only(self, forward_only):
        self.forward_movement_only = forward_only

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX*diffX + diffY*diffY)

    def at_goal(self, cur, goal):
        if goal is None:
            return True
        d = self.get_goal_distance(cur, goal)
        dTh = abs(self.normalize_pi(cur.theta - goal.theta))
        return d < self.linear_tolerance and dTh < self.angular_tolerance

    def get_velocity(self, cur, goal, dT):
        desired = Pose()

        goal_heading = atan2(goal.y - cur.y, goal.x - cur.x)
        a = -cur.theta + goal_heading

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta = self.normalize_pi(cur.theta - goal.theta)
        b = -theta - a

        # rospy.loginfo('cur=%f goal=%f a=%f b=%f', cur.theta, goal_heading,
        #               a, b)

        d = self.get_goal_distance(cur, goal)
        if self.forward_movement_only:
            direction = 1
            a = self.normalize_pi(a)
            b = self.normalize_pi(b)
        else:
            direction = self.sign(cos(a))
            a = self.normalize_half_pi(a)
            b = self.normalize_half_pi(b)

        # rospy.loginfo('After normalization, a=%f b=%f', a, b)

        if abs(d) < self.linear_tolerance:
            desired.xVel = 0
            desired.thetaVel = self.kB * theta
        else:
            desired.xVel = self.kP * d * direction
            desired.thetaVel = self.kA*a + self.kB*b

        # Adjust velocities if X velocity is too high.
        if abs(desired.xVel) > self.max_linear_speed:
            ratio = self.max_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired.thetaVel) > self.max_angular_speed:
            ratio = self.max_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        # Adjust velocities if too low, so robot does not stall.
        if abs(desired.xVel) > 0 and abs(desired.xVel) < self.min_linear_speed:
            ratio = self.min_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio
        elif desired.xVel==0 and abs(desired.thetaVel) < self.min_angular_speed:
            ratio = self.min_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        return desired

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > pi/2:
            return alpha - pi
        elif alpha < -pi/2:
            return alpha + pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2*pi
        while alpha < -pi:
            alpha += 2*pi
        return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
