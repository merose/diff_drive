from __future__ import division
from math import pi, sin, cos
from diff_drive.encoder import Encoder
from diff_drive.pose import Pose

class Odometry:
    """Keeps track of the current position and velocity of a
    robot using differential drive.
    """

    def __init__(self):
        self.leftEncoder = Encoder()
        self.rightEncoder = Encoder()
        self.pose = Pose()
        self.lastTime = 0

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks
        
    def setEncoderRange(self, low, high):
        self.leftEncoder.setRange(low, high)
        self.rightEncoder.setRange(low, high)

    def setTime(self, newTime):
        self.lastTime = newTime
        
    def updateLeftWheel(self, newCount):
        self.leftEncoder.update(newCount)

    def updateRightWheel(self, newCount):
        self.rightEncoder.update(newCount)

    def updatePose(self, newTime):
        leftTravel = self.leftEncoder.getDelta() / self.ticksPerMeter
        rightTravel = self.rightEncoder.getDelta() / self.ticksPerMeter
        deltaTime = newTime - self.lastTime

        deltaTravel = (leftTravel + rightTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.wheelSeparation
        meanTheta = self.pose.theta + deltaTheta/2

        deltaX = deltaTravel * cos(meanTheta)
        deltaY = deltaTravel * sin(meanTheta)

        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = (self.pose.theta + deltaTheta) % (2*pi)
        self.pose.xVel = deltaX / deltaTime
        self.pose.yVel = deltaY / deltaTime
        self.pose.thetaVel = deltaTheta / deltaTime

        self.lastTime = newTime

    def getPose(self):
        return self.pose;

    def setPose(self, newPose):
        pose = newPose
