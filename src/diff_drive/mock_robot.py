from __future__ import division


class MotorTicks:

    def __init__(self):
        self.left = 0
        self.right = 0

class MockRobot:
    """Implements a mock robot that dutifully executes its wheel
    speed commands exactly.
    """

    def __init__(self):
        self.leftSpeed = 0
        self.newLeftSpeed = 0
        self.rightSpeed = 0
        self.newRightSpeed = 0
        self.leftTicks = 0
        self.rightTicks = 0
        self.minTicks = -32768
        self.maxTicks = 32767

    def setSpeeds(self, left, right):
        self.newLeftSpeed = left
        self.newRightSpeed = right

    def updateRobot(self, dTime):
        self.leftTicks = self.addTicks(self.leftTicks, self.leftSpeed*dTime)
        self.rightTicks = self.addTicks(self.rightTicks, self.rightSpeed*dTime)
        self.leftSpeed = self.newLeftSpeed
        self.rightSpeed = self.newRightSpeed

    def getTicks(self):
        ticks = MotorTicks()
        ticks.left = self.leftTicks
        ticks.right = self.rightTicks
        return ticks

    def addTicks(self, ticks, deltaTicks):
        ticks += deltaTicks
        if ticks > self.maxTicks:
            return int(ticks - self.maxTicks + self.minTicks)
        elif ticks < self.minTicks:
            return int(ticks - self.minTicks + self.maxTicks)
        else:
            return int(ticks)
