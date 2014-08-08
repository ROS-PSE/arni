PKG = 'arni_countermeasure'
NAME = 'constraint_test'

import unittest

from arni_core.helper import SEUID_DELIMITER

from constraint_and import *
from constraint_item import *
from constraint_leaf import *
from constraint_or import *
from constraint import *
from rated_statistic_storage import *
from arni_msgs import *
import rospy


class TestConstraint(unittest.TestCase):

    def test_simple_tree(self):
        """Test if an constraint tree can be build and evaluates correctly.
        """
        storage = RatedStatisticStorage()
        storage.callback_rated_statistic(
            self._genMessage("node1", "ram_usage_mean", ["20"], [chr(Outcome.HIGH)]))
        storage.callback_rated_statistic(
            self._genMessage(
                "node2", "node_cpu_usage_mean", ["40"], [chr(Outcome.OUT_OF_BOUNDS)]))

        cleaf = ConstraintLeaf(
            "n%snode1" % SEUID_DELIMITER, "ram_usage_mean",
            Outcome.OUT_OF_BOUNDS)

        cleaf2 = ConstraintLeaf(
            "n%snode2" % SEUID_DELIMITER, "node_cpu_usage_mean",
            Outcome.OUT_OF_BOUNDS)

        cand = ConstraintAnd([cleaf, cleaf2])

        self.assertTrue(cand.evaluate_constraint(storage))

    def _genMessage(self, node, statistic_type, value, outcome):
        """Generate a simple message."""
        msg = RatedStatistics()
        msg.seuid = "n%s%s" % (SEUID_DELIMITER, node)
        msg.window_start = rospy.Time().now() - rospy.Duration(1)
        msg.window_stop = rospy.Time().now()

        msgEntity = RatedStatisticsEntity()
        msgEntity.statistic_type = statistic_type
        msgEntity.actual_value = value
        msgEntity.state = outcome

        msg.rated_statistics_entity = [msgEntity, ]
        return msg





if __name__ == '__main__':
    # todo: rosunit.unitrun...
    rospy.init_node("test_constraint")
    unittest.main()
