#!/usr/bin/env python
import unittest
from arni_countermeasure.rated_statistic_storage import *
from rosgraph_msgs.msg import Clock
from arni_countermeasure.outcome import *
import rospy
import time
import arni_countermeasure.helper as helper
PKG = "arni_countermeasure"


class TestStorage(unittest.TestCase):

    pub = None

    @classmethod
    def setUpClass(TestStorage):
        rospy.set_param("/use_sim_time", "true")
        TestStorage.pub = rospy.Publisher('clock', Clock, queue_size=10)

        rospy.init_node("test storage", anonymous=True)
        TestStorage.set_time(10)

    def setUp(self):
        TestStorage.set_time(10)

    def test_add_old(self):
        """Test if adding an too old statistic will not be saved."""
        TestStorage.set_timeout(10)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        store._RatedStatisticStorage__add_single_outcome(
            "n!node1", "cpu", Outcome.HIGH, rospy.Time(90))
        self.assertEqual(
            store.get_outcome("n!node1", "cpu"), Outcome.UNKNOWN)

        """Test if an statistic thats just not to old will be saved."""
        TestStorage.set_timeout(10)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        store._RatedStatisticStorage__add_single_outcome(
            "n!node2", "cpu", Outcome.HIGH, rospy.Time(91))
        self.assertEqual(
            store.get_outcome("n!node2", "cpu"), Outcome.HIGH)

    def test_remove_through_timeout(self):
        """Test that an statistic is not present after timeout."""
        TestStorage.set_timeout(20)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        store._RatedStatisticStorage__add_single_outcome(
            "n!node3", "cpu", Outcome.HIGH, rospy.Time(100))
        self.assertEqual(
            store.get_outcome("n!node3", "cpu"), Outcome.HIGH)
        TestStorage.set_time(120)
        self.assertEqual(
            store.get_outcome("n!node3", "cpu"), Outcome.UNKNOWN)

    def test_add_new_than_old(self):
        """Test adding a statistic and then another one of the same
        type but older."""
        TestStorage.set_timeout(20)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        store._RatedStatisticStorage__add_single_outcome(
            "n!node4", "cpu", Outcome.HIGH, rospy.Time(100))
        store._RatedStatisticStorage__add_single_outcome(
            "n!node4", "cpu", Outcome.LOW, rospy.Time(90))

        self.assertEqual(
            store.get_outcome("n!node4", "cpu"), Outcome.HIGH)

    def test_callback_single_entry(self):
        """Test callback with a statistic type holding only one entry."""
        TestStorage.set_timeout(20)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        entity_c = TestStorage._gen_entity(
            "ram_usage_mean", ["20"], [chr(Outcome.HIGH)])
        msg = TestStorage._gen_msg("n!node", 100, [entity_c])
        store.callback_rated_statistic(msg)

        self.assertEqual(
            store.get_outcome("n!node", "ram_usage_mean"), Outcome.HIGH)

    def test_callback_multiple_entries(self):
        """Test callback with a statistic type holding multiple entries."""
        TestStorage.set_timeout(20)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        entity_c = TestStorage._gen_entity(
            "ram_usage_mean", ["20", "40"],
            [chr(Outcome.HIGH), chr(Outcome.LOW)])
        msg = TestStorage._gen_msg("n!node", 100, [entity_c])
        store.callback_rated_statistic(msg)

        self.assertEqual(
            store.get_outcome("n!node", "ram_usage_mean_0"), Outcome.HIGH)
        self.assertEqual(
            store.get_outcome("n!node", "ram_usage_mean_1"), Outcome.LOW)

    def test_callback_multiple_rated_entities(self):
        """Test a callback with multiple entities."""
        TestStorage.set_timeout(20)
        TestStorage.set_time(100)
        store = RatedStatisticStorage()
        entity_c = TestStorage._gen_entity(
            "ram_usage_mean", ["20"], [chr(Outcome.HIGH)])
        entity_c2 = TestStorage._gen_entity(
            "ram_usage_max", ["80"], [chr(Outcome.LOW)])
        msg = TestStorage._gen_msg("n!node", 100, [entity_c, entity_c2])
        store.callback_rated_statistic(msg)

        self.assertEqual(
            store.get_outcome("n!node", "ram_usage_mean"), Outcome.HIGH)
        self.assertEqual(
            store.get_outcome("n!node", "ram_usage_max"), Outcome.LOW)

    @classmethod
    def _gen_entity(TestStorage, statistic_type, value, outcome):
        msgEntity = RatedStatisticsEntity()
        msgEntity.statistic_type = statistic_type
        msgEntity.actual_value = value
        msgEntity.state = outcome
        return msgEntity

    @classmethod
    def _gen_msg(TestStorage, seuid, time_arrive, entity_container):
        msg = RatedStatistics()
        msg.seuid = seuid
        msg.window_start = rospy.Time(time_arrive - 1)
        msg.window_stop = rospy.Time(time_arrive)
        msg.rated_statistics_entity = entity_container
        return msg

    @classmethod
    def set_timeout(TestStorage, timeout):
        rospy.set_param(helper.ARNI_CTM_CFG_NS + "storage_timeout", timeout)

    @classmethod
    def set_time(TestStorage, value):
        wanted_time = rospy.Time(value)
        c = Clock()
        c.clock = wanted_time
        while rospy.Time.now() != wanted_time:
            TestStorage.pub.publish(c)
            time.sleep(0.01)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_storage', TestStorage)
