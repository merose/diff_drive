from __future__ import division

class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xVel = 0
        self.yVel = 0
        self.thetaVel = 0


    def __str__(self):
        return str({'x': self.x, 'y': self.y, 'theta': self.theta,
                    'xVel': self.xVel, 'yVel': self.yVel,
                    'thetaVel': self.thetaVel})
