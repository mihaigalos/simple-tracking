"""
Simple object tracking implemented in Python 2 using ROS and PyKalman.
"""
#!/usr/bin/env python

import collections  # for deque

import rviz_tools_py as rviz_tools
import rospy

from cinematic_models import ConstantAccelerationPoseCalculator
from kalman_wrappers import KalmanWrapperCircularConstantAcceleration


class Tracking(object):
    """
    Actual tracking implementation.
    """

    def __init__(self, cinematic_model, kalman_wrapper):

        self.markers = None
        self.update_frecuency = 100.0
        self.cinematic_model = cinematic_model
        self.kalman_wrapper = kalman_wrapper

        rospy.init_node('tracking', anonymous=False,
                        log_level=rospy.INFO, disable_signals=False)
        rospy.on_shutdown(self.cleanup_node)
        self.markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

        self.deque = collections.deque(maxlen=2)
        self.i = 0

    def cleanup_node(self):
        """
        Teardown method.
        """
        print "Shutting down node"
        self.markers.deleteAllMarkers()

    def run(self):
        """
        Periodic update object position and tracking.
        """
        while not rospy.is_shutdown():
            new_pose = self.cinematic_model.get_new_pose()
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


Tracking(ConstantAccelerationPoseCalculator(),
         KalmanWrapperCircularConstantAcceleration()).run()
