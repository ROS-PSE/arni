#!/usr/bin/env python
import unittest

import rospy

from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from arni_countermeasure.outcome import *
from arni_core.helper import *
from rosgraph_msgs.msg import Log
import time
import sys

PKG = "arni_countermeasure"


class TestScenario(unittest.TestCase):

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
        r = rospy.Rate(10)  # hz
        begin_time = rospy.Time.now()

        while (
                (not rospy.is_shutdown())
                and rospy.Time.now() - begin_time <= rospy.Duration(10)):
            entity = self.create_statistic_entity(
                "cpu_usage_max", ["90"], ["0-50"], [Outcome.HIGH])

            msg = self.create_msg(
                "n%snode1" % SEUID_DELIMITER, "127.0.0.1", [entity])
            self.pub.publish(msg)
            r.sleep()

        rospy.Rate(0.1).sleep()

        # check if the constrain has been fulfilled
        print "test1 %s" % self.log
        self.assertIn(
            "cpu high test1", self.log,
            "reaction after high cpu did not get executed.")

    def test_high_cpu_too_short(self):
        """ Test for no reaction after having high cpu to short."""
        r = rospy.Rate(10)  # hz
        begin_time = rospy.Time.now()

        while (
                (not rospy.is_shutdown())
                and rospy.Time.now() - begin_time <= rospy.Duration(3)):
            entity = self.create_statistic_entity(
                "cpu_usage_max", ["90"], ["0-50"], [Outcome.HIGH])

            msg = self.create_msg(
                "n%snode2" % SEUID_DELIMITER, "127.0.0.1", [entity])
            self.pub.publish(msg)
            r.sleep()

        # sleep for 10 simulated seconds.
        rospy.Rate(0.1).sleep()
        # check if the constrain has been fulfilled
        print "test2, %s" % self.log
        self.assertNotIn(
            "cpu high test2", self.log,
            "storage_timeout should have been to small for"
            + " the constraint to be true long enough.")

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
