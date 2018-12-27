"""
Wrappers for different types of Kalman filters, depending on the cinematic models.
"""

from pykalman import KalmanFilter


class KalmanWrapperCircularConstantAcceleration(object):
    """
    Wrapper for configuring and passing values to and from the Kalman KalmanFilter.
    """

    def __init__(self):
        """
        Transition matrix has form [x, x_dot, y, y_dot].
        Observation matrix has same form.
        """
        self.transition_matrix = [[1, 1, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 1],
                                  [0, 0, 0, 1]]

        self.observation_matrix = [[1, 0, 0, 0],
                                   [0, 0, 1, 0]]

    def make_prediction(self, measurements):
        """
        Compute prediction of object position for the next step.
        """
        initial_state_mean = [measurements[0][0], 0,
                              measurements[0][1], 0]

        kf1 = KalmanFilter(transition_matrices=self.transition_matrix,
                           observation_matrices=self.observation_matrix,
                           initial_state_mean=initial_state_mean)
        kf1 = kf1.em(list(measurements), n_iter=2)
        return kf1.smooth(measurements)

    def get_prediction(self, measurements):
        """
        Compute object position prediction based on current state and Kalman filter's
        parameters.
        """
        prediction = self.make_prediction(measurements)
        return prediction[0]
