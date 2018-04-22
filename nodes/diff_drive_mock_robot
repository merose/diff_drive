#! /usr/bin/env python
from __future__ import division

import rospy
from math import pi, asin
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from diff_drive import mock_robot

class MockRobotNode:

    def __init__(self):
        self.robot = mock_robot.MockRobot()
        self.leftSpeed = 0
        self.rightSpeed = 0

    def main(self):
        rospy.init_node('diff_drive_mock_robot')
        self.leftPub = rospy.Publisher('~lwheel_ticks',
                                       Int32, queue_size=10)
        self.rightPub = rospy.Publisher('~rwheel_ticks',
                                        Int32, queue_size=10)

        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber('~lwheel_desired_rate', Int32, self.leftCallback)
        rospy.Subscriber('~rwheel_desired_rate', Int32,
                         self.rightCallback)

        self.rate = rospy.get_param('~rate', 10.0)
        self.timeout = rospy.get_param('~timeout', 0.5)

        rate = rospy.Rate(self.rate)
        self.lastTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        newTime = rospy.get_time()
        diffTime = newTime - self.lastTime
        self.lastTime = newTime

        if diffTime > self.timeout:
            self.robot.setSpeeds(0, 0)
    
        try:
            self.robot.updateRobot(diffTime)
        except:
            rospy.logerror("Got exception updating robot")
        ticks = self.robot.getTicks()
        self.leftPub.publish(ticks.left)
        self.rightPub.publish(ticks.right)

    def leftCallback(self, leftSpeed):
        self.leftSpeed = leftSpeed.data
        self.robot.setSpeeds(self.leftSpeed, self.rightSpeed)

    def rightCallback(self, rightSpeed):
        self.rightSpeed = rightSpeed.data
        self.robot.setSpeeds(self.leftSpeed, self.rightSpeed)


if __name__ == '__main__':
    try:
        node = MockRobotNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
