#! /usr/bin/env python
from __future__ import division
PKG='test_odometry'

from math import pi, sin, cos
import unittest
from diff_drive.odometry import Odometry


class TestOdometry(unittest.TestCase):

    def setUp(self):
        self.wheelSeparation = 0.10
        self.ticksPerMeter = 10000
        self.odom = Odometry()
        self.odom.setWheelSeparation(self.wheelSeparation)
        self.odom.setTicksPerMeter(self.ticksPerMeter)

    def testInitialization(self):
        pose = self.odom.getPose()
        self.assertEquals(pose.x, 0)
        self.assertEquals(pose.y, 0)
        self.assertEquals(pose.theta, 0)
        self.assertEquals(pose.xVel, 0)
        self.assertEquals(pose.yVel, 0)
        self.assertEquals(pose.thetaVel, 0)

    def testTravelForward(self):
        self.checkUpdate(10000, 10000, 2, {'x': 1, 'xVel': 1/2})

    def testSpinLeft(self):
        angle = (200/self.ticksPerMeter / self.wheelSeparation) % (2*pi)
        self.checkUpdate(-100, 100, 2,
                         {'theta': angle, 'thetaVel': angle/2})

    def testSpinRight(self):
        angle = (200/self.ticksPerMeter / self.wheelSeparation) % (2*pi)
        self.checkUpdate(100, -100, 2,
                         {'theta': (-angle) % (2*pi),
                          'thetaVel': -angle/2})

    def testCurveLeft(self):
        radius = self.wheelSeparation / 2
        angle = pi
        s = angle * self.wheelSeparation
        ticks = int(s * self.ticksPerMeter)
        self.checkUpdate(0, ticks, 2,
                         {'x': 0,
                          'y': 2*radius,
                          'theta': angle,
                          'xVel': s/4,
                          'yVel': 0,
                          'thetaVel': angle/2})
                          
    def testCurveRight(self):
        radius = self.wheelSeparation / 2
        angle = pi
        s = angle * self.wheelSeparation
        ticks = int(s * self.ticksPerMeter)
        self.checkUpdate(ticks, 0, 2,
                         {'x': 0,
                          'y': -2*radius,
                          'theta': -angle,
                          'xVel': s/4,
                          'yVel': 0,
                          'thetaVel': -angle/2})
                          
    def testCircle(self):
        self.checkCircleRight(8, 9)
        self.checkCircleRight(0, 100)
        self.checkCircleRight(100, 0)

    def checkCircle(self, vr, vl):
        radius = abs(self.wheelSeparation/2 * (vr+vl)/(vr-vl))
        circumference = 2*pi*radius
        deltaTravel = (vr+vl)/2 * self.ticksPerMeter;
        for i in range(int(circumference/deltaTravel)):
            self.odom.updateLeftWheel(vl)
            self.odom.updateRightWheel(vr)
            self.odom.updatePose(i+1)
        self.checkPose(self.odom.getPose(), {'x': 0, 'y': 0})

    def checkUpdate(self, leftTicks, rightTicks, deltaTime, attrs):
        self.odom.updateLeftWheel(leftTicks)
        self.odom.updateRightWheel(rightTicks)
        self.odom.updatePose(deltaTime)
        self.checkPose(self.odom.getPose(), attrs)
        
    def checkPose(self, pose, attrs):
        for key in ['x', 'y', 'theta', 'xVel', 'yVel', 'thetaVel']:
            if key in attrs:
                self.assertClose(
                    getattr(pose, key), attrs[key],
                            msg="{0}: {1}!={2}".format(key,
                                                    getattr(pose, key),
                                                    attrs[key]))
            else:
                self.assertEquals(getattr(pose, key), 0, key)

    def assertClose(self, x, y, msg):
        if y == 0:
            self.assertLess(abs(x), 0.0001, msg)
        else:
            self.assertLess(abs(x-y)/y, 0.001, msg)
        
if __name__ == '__main__':
    unittest.main()
