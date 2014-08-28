#!/usr/bin/env python
import unittest

import rospy

from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from arni_countermeasure.outcome import *
from arni_core.helper import *
from rosgraph_msgs.msg import Log
import time

PKG = "arni_countermeasure"


class TestScenario(unittest.TestCase):
    """Test a couple of simple scenarios.

    Usually the countermeasure node gets some rated statistics and
    acts upon some constraints."""
    @classmethod
    def setUpClass(TestScenario):
        pass

    def setUp(self):
        self.log = list()
        self.sub = rospy.Subscriber(
            '/rosout', Log, self.log_callback)
        self.pub = rospy.Publisher(
            '/statistics_rated', RatedStatistics, queue_size=10)
        rospy.init_node('talker', anonymous=True)

        # wait for initializing
        while (rospy.Time.now() == rospy.Time(0)):
            time.sleep(0.01)
        self.log = list()
        rospy.Rate(0.1).sleep()

    def tearDown(self):
        pass

    def log_callback(self, msg):
        self.log.append(msg.msg)

    def test_high_cpu(self):
        """ Test reacting to high cpu."""

        self.create_default_msg("node1", Outcome.HIGH, 15)
        rospy.Rate(0.1).sleep()

        # check if the constrain has been fulfilled
        self.assertIn(
            "cpu high test1", self.log)
        # "reaction after high cpu did not get executed."

    def test_high_cpu_too_short(self):
        """ Test for no reaction after having high cpu to short."""
        self.create_default_msg("node2", Outcome.HIGH, 3)

        # sleep for 10 simulated seconds.
        rospy.Rate(0.1).sleep()
        # check if the constrain has been fulfilled
        self.assertNotIn(
            "cpu high test2", self.log,
            "storage_timeout should have been to small for"
            + " the constraint to be true long enough.")

    def test_constraint_timeout(self):
        """Test the timeout after having executed a reaction."""

        self.create_default_msg("node3", Outcome.LOW, 15)
        rospy.Rate(0.1).sleep()

        self.assertIn("test3", self.log, "first execution didn't work.")

        self.log = list()
        self.create_default_msg("node3", Outcome.LOW, 35)
        rospy.Rate(0.1).sleep()

        self.assertNotIn(
            "test3", self.log,
            "got a second execution that shouldn't happen.")

    def test_reaction_autonomy_level_too_high(self):
        """ Test for no reaction because of a too high autonomy_level."""
        self.create_default_msg("node4", Outcome.LOW, 20)
        self.assertNotIn("test4", self.log)

    def create_default_msg(self, node, outcome, durotation):
        """ Create an default message for durotation seconds.
        """
        r = rospy.Rate(3)  # hz
        begin_time = rospy.Time.now()

        while (
                (not rospy.is_shutdown())
                and rospy.Time.now() - begin_time <= rospy.Duration(
                    durotation)):
            entity = self.create_statistic_entity(
                "cpu_usage_max", ["90"], ["0-50"], [outcome])

            msg = self.create_msg(
                "n%s%s" % (SEUID_DELIMITER, node), "127.0.0.1", [entity])
            self.pub.publish(msg)
            r.sleep()

    def create_msg(self, seuid, host, entity):
        msg = RatedStatistics()
        msg.seuid = seuid
        msg.host = host

        now = rospy.Time.now()
        msg.window_start = now - rospy.Duration(1)
        msg.window_stop = now

        msg.rated_statistics_entity = entity
        return msg

    def create_statistic_entity(self, s_type, actual, expected, state):
        msg = RatedStatisticsEntity()
        msg.statistic_type = s_type
        msg.actual_value = actual
        msg.expected_value = expected
        msg.state = state
        return msg

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_scenario', TestScenario)
