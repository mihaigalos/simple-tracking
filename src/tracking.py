#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import rviz_tools_py as rviz_tools
import math
from tf.transformations import quaternion_from_euler
from pprint import pprint
from pykalman import KalmanFilter
import numpy as np
import collections  # for deque


theta = 0
radius = 1.5


class KalmanWrapper:
    def __init__(self, initial_state_mean):
        self.transition_matrix = [[1, 1, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 1],
                                  [0, 0, 0, 1]]

        self.observation_matrix = [[1, 0, 0, 0],
                                   [0, 0, 1, 0]]

    def get_prediction(self, measurements):

        initial_state_mean = [measurements[0][0], 0,
                              measurements[0][1], 0]

        kf1 = KalmanFilter(transition_matrices=self.transition_matrix,
                           observation_matrices=self.observation_matrix,
                           initial_state_mean=initial_state_mean)
        kf1 = kf1.em(list(measurements), n_iter=2)
        (smoothed_state_means, smoothed_state_covariances) = kf1.smooth(measurements)
        return smoothed_state_means


class PoseCalculator:

    def get_new_pose(self):
        global radius
        global theta
        theta = theta + 0.01
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        z = 0

        quaternion = quaternion_from_euler(theta, 0, theta)
        P = Pose(Point(x, y, z), Quaternion(
            quaternion[0], quaternion[1], 0, 0))

        print "Current pose:"
        pprint(P)
        print "\n"
        return P


class Wrapper:
    def __init__(self):

        self.markers = None
        self.update_frecuency = 100.0

        rospy.init_node('test', anonymous=False,
                        log_level=rospy.INFO, disable_signals=False)
        rospy.on_shutdown(self.cleanup_node)
        self.markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

        global radius
        self.kalman_wrapper = KalmanWrapper([radius, 0, 0, 0])
        self.deque = collections.deque(maxlen=2)
        self.i = 0

    def cleanup_node(self):
        print "Shutting down node"
        self.markers.deleteAllMarkers()

    def run(self):
        while not rospy.is_shutdown():
            new_pose = PoseCalculator().get_new_pose()
            cube_width = 0.6
            # pose, color, cube_width, lifetime
            self.markers.publishCube(new_pose, 'green', cube_width,
                                     1 / self.update_frecuency)

            self.deque.append([new_pose.position.x, new_pose.position.y])
            if self.i > 2:
                tracked_pose = new_pose
                prediction = self.kalman_wrapper.get_prediction(self.deque)

                tracked_pose.position.x = prediction[0][0]
                tracked_pose.position.y = prediction[0][2]

                self.markers.publishCube(tracked_pose, 'translucent',
                                         cube_width + 0.1, 1 / self.update_frecuency)
            self.i = self.i + 1

            rospy.Rate(self.update_frecuency).sleep()


wrapper = Wrapper()
wrapper.run()
