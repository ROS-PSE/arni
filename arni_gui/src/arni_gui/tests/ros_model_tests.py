__author__ = 'virtubuntu'

from arni_gui.ros_model import ROSModel
import unittest
from python_qt_binding.QtCore import QObject

class ROSModelTest(unittest.TestCase):

    def test_singleton_property(self):
        """
        Simple test to check if rosmodel is really a singleton.
        """
        model1 = ROSModel()
        model2 = ROSModel()
        self.assertEqual(type(model1), type(model2))

    def check_the_init(self):
        """
        Check if the __init__ is only called once
        """
        model1 = ROSModel(QObject())
        model2 = ROSModel(None)
        self.assertEqual(model1.__parent, None)

unittest.main()