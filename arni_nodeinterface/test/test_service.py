#!/usr/bin/env python

import unittest
from arni_nodeinterface.host_statistics_handler import *
from arni_nodeinterface.host_status import *
from arni_nodeinterface.node_statistics_handler import *
from arni_msgs.srv import NodeReaction

import traceback
import rospy

PKG = 'arni_nodeinterface'

class  TestService(unittest.TestCase):

    def test_kill(self):
        rospy.sleep(rospy.Duration(10))
        stop_node = rospy.ServiceProxy('/execute_node_reaction/192_168_0_14', NodeReaction)
        rtn = stop_node('/writer', 'stop', '')
        self.assertIn(' successfully stopped', str(rtn))

    def test_restart(self):
        rospy.sleep(rospy.Duration(10))
        restart_node = rospy.ServiceProxy('/execute_node_reaction/192_168_0_14', NodeReaction)
        rtn = restart_node('/writer2', 'restart', '')
        self.assertIn('Restarted /writer2', str(rtn))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_service', TestService)