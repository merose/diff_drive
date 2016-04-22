#! /usr/bin/env python
from __future__ import division
PKG='test_odometry'

from math import pi, sin, cos
import unittest
from diff_drive.odometry import Odometry


class TestOdometry(unittest.TestCase):

    def setUp(self):
        self.wheelSeparation = 0.10
        self.ticksPerMeter = 1000
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
        self.checkUpdate(1000, 1000, 2, {'x': 1, 'xVel': 1/2})

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
        distance = (100 + 200)/2 / self.ticksPerMeter
        angle = 100/self.ticksPerMeter / self.wheelSeparation
        self.checkUpdate(100, 200, 2,
                         {'x': distance*cos(angle/2),
                          'y': distance*sin(angle/2),
                          'theta': angle,
                          'xVel': distance*cos(angle/2)/2,
                          'yVel': distance*sin(angle/2)/2,
                          'thetaVel': angle/2})
                          
    def checkUpdate(self, leftTicks, rightTicks, deltaTime, attrs):
        self.odom.updateLeftWheel(leftTicks)
        self.odom.updateRightWheel(rightTicks)
        self.odom.updatePose(deltaTime)
        self.checkPose(self.odom.getPose(), attrs)
        
    def checkPose(self, pose, attrs):
        for key in ['x', 'y', 'theta', 'xVel', 'yVel', 'thetaVel']:
            if key in attrs:
                self.assertAlmostEqual(
                    getattr(pose, key), attrs[key], 7,
                            "{0}: {1}!={2}".format(key,
                                                   getattr(pose, key),
                                                   attrs[key]))
            else:
                self.assertEquals(getattr(pose, key), 0, key)

if __name__ == '__main__':
    unittest.main()
