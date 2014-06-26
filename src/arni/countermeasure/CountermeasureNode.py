from ConstraintHandler import ConstraintHandler
from RatedStatisticStorage import RatedStatisticStorage


class CountermeasureNode(object):

    """A ROS node. Evaluates incoming rated statistics with a list of constraints. If those constraints turn out to be true appropriate action is taken."""

    def __init__(self):
        super(CountermeasureNode, self).__init__()
        constraint_handler = ConstraintHandler()
        rated_statistic_storage = RatedStatisticStorage()


def main():
    cn = CountermeasureNode()

if __name__ == '__main__':
    main()
