from constraint_handler import *
from rated_statistic_storage import *

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
        self.__constraint_handler = ConstraintHandler()

        #: The storage of all incoming rated statistic.
        self.__rated_statistic_storage = RatedStatisticStorage()

    def __register_subscriber(self):
        """Register to the rated statistics."""
        pass


def main():
    cn = CountermeasureNode()

if __name__ == '__main__':
    main()
