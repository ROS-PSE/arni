from threading import Lock, Thread

from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration
from rospy.rostime import Time
import rospy
from rospy.service import ServiceException

from rosgraph_msgs.msg import TopicStatistics
from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics

from arni_msgs.srv import StatisticHistory

from ros_model import *
from helper_functions import UPDATE_FREQUENCY


class BufferThread(Thread):
    """
    This thread should buffer the incoming data and regularly update the model and hence also the model.
    """

    def __init__(self, model):
        """
        Initializes the BufferThread

        :param model: the object of the ROSModel
        :type model: ROSModel
        """
        super(BufferThread, self).__init__()

        self.__rated_statistics_buffer = list()
        self.__topic_statistics_buffer = list()
        self.__node_statistics_buffer = list()
        self.__host_statistics_buffer = list()
        self.__running = False
        self.__model = model
        self.start()
        self.__timer = Timer(Duration(nsecs=UPDATE_FREQUENCY), self.__update_model)


    def __del__(self):
        """
        The Destructor of the BufferThread
        """
        # todo: I think this does not work as intended :(
        self.__timer.stop()
        del self.__timer


    def start(self):
        if not self.__running:
            self.__get_history()
            self.__register_subscribers()
            self.__running = True


    def __get_history(self):
        """
        For fetching the history from the monitoring_node.
        """
        try:
            get_statistic_history = rospy.ServiceProxy('monitoring_node/get_statistic_history', StatisticHistory)
            response = get_statistic_history(rospy.Time(0))
            rated_statistics_history = response.rated_topic_statistics + response.rated_node_statistics + response.rated_host_statistics + response.rated_node_statistics
            self.__model.update_model(rated_statistics_history, response.topic_statistics,
                                      response.host_statistics, response.node_statistics)
        except ServiceException as msg:
            self.__model.get_logger().log("info", Time.now(), "BufferThread",
                                          "get_statistic_history is not available, probably monitoring_node is not running. Will continue without the information about the past")


    def __register_subscribers(self):
        """
        Registers to the services needed to get fresh data.
        """
        rospy.Subscriber(
            "/statistics_rated", RatedStatistics,
            self.__add_rated_statistics_item)
        rospy.Subscriber(
            "/statistics", TopicStatistics,
            self.__add_topic_statistics_item)
        rospy.Subscriber(
            "/statistics_node", NodeStatistics,
            self.__add_node_statistics_item)
        rospy.Subscriber(
            "/statistics_host", HostStatistics,
            self.__add_host_statistics_item)


    def __update_model(self, event):
        """
        Starts the update of the model. 
        Will be called regulary by the timer, first read the data from the *buffer* and add the according data items to the items of the model,
        afterwards use the *rated_buffer* to add a rating to these entries.
        """
        self.__model.update_model(self.__rated_statistics_buffer, self.__topic_statistics_buffer,
                                  self.__host_statistics_buffer, self.__node_statistics_buffer)

        del self.__rated_statistics_buffer[:]
        del self.__topic_statistics_buffer[:]
        del self.__host_statistics_buffer[:]
        del self.__node_statistics_buffer[:]


    def __add_rated_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: RatedStatistics
        """
        self.__rated_statistics_buffer.append(item)


    def __add_topic_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: TopicStatistics
        """
        self.__topic_statistics_buffer.append(item)


    def __add_node_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: NodeStatistics
        """
        self.__node_statistics_buffer.append(item)


    def __add_host_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: HostStatistics
        """
        self.__host_statistics_buffer.append(item)