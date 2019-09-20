import unittest
from src.cinematic_models import ConstantAccelerationPoseCalculator
import collections  # for deque


class ParsingWorks(unittest.TestCase):
    def setUp(self):
        self.unit = ConstantAccelerationPoseCalculator()

    def test_whenTypical(self):
        self.unit.get_new_pose()
        self.assertEquals(self.unit.radius, 1.6)
        self.assertEquals(self.unit.theta, 0.01)


if __name__ == '__main__':
    unittest.main()
