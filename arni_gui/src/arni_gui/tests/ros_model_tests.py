__author__ = 'virtubuntu'

from arni_gui.ros_model import ROSModel
import unittest


class ROSModelTest(unittest.TestCase):

    def test_singleton_property(self):
        """
        Simple test to check if rosmodel is really a singleton.
        """
        model1 = ROSModel()
        model2 = ROSModel()
        self.assertEqual(type(model1), type(model2))

unittest.main()