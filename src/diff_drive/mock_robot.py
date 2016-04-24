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

    def setSpeeds(self, left, right):
        self.newLeftSpeed = left
        self.newRightSpeed = right

    def updateRobot(self, diffTime):
        self.leftTicks += self.leftSpeed * diffTime
        self.rightTicks += self.rightSpeed * diffTime
        self.leftSpeed = self.newLeftSpeed
        self.rightSpeed = self.newRightSpeed

    def getTicks(self):
        ticks = MotorTicks()
        ticks.left = self.leftTicks
        ticks.right = self.rightTicks
        return ticks
