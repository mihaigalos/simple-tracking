"""
Different cinematic models for calculating poses at each iteration.
"""

import math
from pprint import pprint
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class ConstantAccelerationPoseCalculator(object):
    """
    Circular cinematic model based on constant acceleration.
    """

    def __init__(self):
        self.radius = 1.6
        self.theta = 0
        self.last_pose = Pose()

    def get_new_pose(self):
        """
        Compute new pose by incrementing the angle along an imaginary circle.
        """

        self.theta = self.theta + 0.01
        x_pos = self.radius * math.cos(self.theta)
        y_pos = self.radius * math.sin(self.theta)
        z_pos = 0

        quaternion = quaternion_from_euler(0, 0, self.theta)
        self.last_pose = Pose(Point(x_pos, y_pos, z_pos), Quaternion(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        return self.last_pose

    def print_last_pose(self):
        """
        Pretty print last pose.
        """

        print "Current pose:"
        pprint(self.last_pose)
        print "\n"
