from abc import ABCMeta, abstractmethod


class StatisticsHandler(object):

    """
    Abstract Class to Handle Statistics of Hosts or Nodes.
    """

    __metaclass__ = ABCMeta

    def __init__(self, _id):

        super(StatisticsHandler, self).__init__()

        # ID of the host or node.
        self._id = _id

        # Holds information about the current status.
        # self._status

    @abstractmethod
    def measure_status(self):
        """
        Periodically collects information about the
        current status.
        """
        pass

    @abstractmethod
    def publish_status(self, topic):
        """
        Publishes the current stats to the given topic
        using ROS's publisher-subscriber mechanism.

        :topic: Topic to which the data should be published
        """
        pass
