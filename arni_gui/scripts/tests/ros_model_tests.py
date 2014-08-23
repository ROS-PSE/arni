#!/usr/bin/env python

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

    def test_log_model(self):
        log_model = ROSModel().get_log_model()


unittest.main()