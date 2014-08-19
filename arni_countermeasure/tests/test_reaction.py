#!/usr/bin/env python
import unittest
import rospy
from arni_core.host_lookup import *
from arni_countermeasure.reaction_restart_node import *
from arni_countermeasure.reaction_stop_node import *
from arni_countermeasure.reaction_publish_rosout_node import *
from arni_countermeasure.reaction_run import *
from rosgraph_msgs.msg import Log
from arni_msgs.srv import NodeReaction, NodeReactionResponse
import re

PKG = "arni_countermeasure"


class TestReaction(unittest.TestCase):

    log = list()

    @classmethod
    def setUpClass(TestReaction):
        # log rosout
        rospy.Subscriber(
            '/rosout', Log, TestReaction.log_callback)
        rospy.init_node('test_node', anonymous=True, log_level=rospy.DEBUG)
        rospy.Service(
            "/execute_node_reaction/192.168.0.1",
            NodeReaction, TestReaction.handle_service)
        rospy.Rate(1).sleep()

    def setUp(self):
        HostLookup().clear()
        TestReaction.log = list()

    def test_reaction_restart(self):
        """Test an simple reaction restart."""
        HostLookup().add_node("node", "192.168.0.1")
        rr = ReactionRestartNode("node", 0)
        rr.execute_reaction()
        # wait a bit to get the response
        rospy.Rate(10).sleep()
        self.assertIn(
            "restarting node node returned: restart-", TestReaction.log)

    def test_restart_no_host(self):
        """Test a reaction with no host in hostlookup."""
        self.simple_execute_reaction(
            ReactionRestartNode("node", 0), "could not restart node.*",)

    def test_shutdown_no_host(self):
        """Test a reaction with no host in hostlookup."""
        self.simple_execute_reaction(
            ReactionStopNode("node", 0), "could not stop node.*")

    def test_run_no_host(self):
        """Test a reaction with no host in hostlookup."""
        self.simple_execute_reaction(
            ReactionRun("node", 0, "command"), "could not run a command.*")

    def test_restart_no_service(self):
        """Test a reaction with no service available."""
        HostLookup().add_node("node", "192.168.0.2")

        self.simple_execute_reaction(
            ReactionRestartNode("node", 0),
            "could not restart node node, service.*")

    def test_shutdown_no_service(self):
        """Test a reaction with no service available."""
        HostLookup().add_node("node", "192.168.0.2")

        self.simple_execute_reaction(
            ReactionStopNode("node", 0), "could not stop node node, service.*")

    def test_run_no_service(self):
        """Test a reaction with no service available."""
        HostLookup().add_node("node", "192.168.0.2")

        self.simple_execute_reaction(
            ReactionRun("node", 0, "command"),
            "could not run a command on node node: service.*")

    def simple_execute_reaction(self, reaction, excpected_log_regex):

        reaction.execute_reaction()
        # wait a bit to get the response
        rospy.Rate(10).sleep()

        assertion = False
        for msg in TestReaction.log:
            if re.search(excpected_log_regex, msg):
                assertion = True
        self.assertTrue(assertion)

    @classmethod
    def log_callback(TestReaction, msg):
        TestReaction.log.append(msg.msg)

    @classmethod
    def handle_service(TestReaction, req):
        return NodeReactionResponse("%s-%s" % (req.action, req.command))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_reaction', TestReaction)
    #unittest.main()
