from constraint_handler import *
from rated_statistic_storage import *
import rospy
from arni_msgs.msg import RatedStatistics


class CountermeasureNode(object):

    """A ROS node.
    Evaluates incoming rated statistics with a list of constraints.
    If those constraints turn out to be true appropriate action is taken.
    """

    def __init__(self):
        """Periodically (threading)
        evaluate the constraints and clean old statistics."""
        super(CountermeasureNode, self).__init__()

        #: The handler for all constraints.
        # self.__constraint_handler = ConstraintHandler()

        #: The storage of all incoming rated statistic.
        self.__rated_statistic_storage = RatedStatisticStorage()

        rospy.init_node("countermeasure_node")
        self.__register_subscriber()

    def __register_subscriber(self):
        """Register to the rated statistics."""
        rospy.Subscriber(
            "/statistics_rated", RatedStatistics,
            self.__rated_statistic_storage.callback_rated_statistic)


def main():
    cn = CountermeasureNode()
    rospy.loginfo(rospy.get_caller_id() + ": im on ")
    rospy.spin()


if __name__ == '__main__':
    main()
