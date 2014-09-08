#!/usr/bin/env python

import unittest
import time
import os.path

import traceback
import rospy

PKG = "arni_core"


class TestNodeShutdown(unittest.TestCase):

    def test_file_exists(self):
        self.assertEqual(os.path.isfile(f), True)


if __name__ == '__main__':
    import rosunit
    f = os.path.abspath("~/Desktop/on_node_stop")
    if os.path.isfile(f):
        os.remove(f)
    time.sleep(28)
    rospy.Timer(15, rosunit.unitrun(PKG, 'test_node_shutdown', TestNodeShutdown), True)
