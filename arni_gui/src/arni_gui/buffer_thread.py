from threading import Lock, Thread

from rosgraph_msgs.msg import TopicStatistics
from arni_msgs.msg import RatedStatistics
from arni_msgs.msg import NodeStatistics
from arni_msgs.msg import HostStatistics

from arni_msgs.srv import StatisticHistory

"""TODO: ROSModel richtig verlinken"""
from ros_model import *
from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration
import rospy

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
        Thread.__init__(self)
        rospy.init_node('arni_gui', log_level=rospy.DEBUG)

        self.__rated_statistics_buffer_lock = Lock()
        self.__topic_statistics_buffer_lock = Lock()
        self.__node_statistics_buffer_lock = Lock()
        self.__host_statistics_buffer_lock = Lock()
        self.__model = model
        self.__timer = Timer(Duration(nsecs=100000000), self.__update_model)
        self.__rated_statistics_buffer = list()
        self.__topic_statistics_buffer = list()
        self.__node_statistics_buffer = list()
        self.__host_statistics_buffer = list()
        self.__running = False

    #todo: is this optimal=?
    def start(self):
        if not self.__running:
            self.__get_history()
            self.__register_subscribers()

    def __get_history(self):
        """
        for fetching the history from the monitoring_node
        :return:
        """
        rospy.logdebug("waiting for service %s", "get_statistic_history")
        rospy.wait_for_service('get_statistic_history')

        get_statistic_history = rospy.ServiceProxy('get_statistic_history', StatisticHistory)

        try:
            response = get_statistic_history()
        except rospy.ServiceException as exc:
            rospy.logdebug(("Service did not process request: " + str(exc)))
            #todo: what shall we do with that exception?
            raise


    def __register_subscribers(self):
        """Register to the services needed to get fresh data"""
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


# def start(self):
#     """
#     Starts the thread and also the timer for regulary updates of the model. It is ensured via the running attribute that this function cannot be called multiple times.
#     """


    def __update_model(self):
        """
        Starts the update of the model. Will be called regulary by the timer. Will first read the data from the *buffer* and add the according data items to the items of the model and afterwards use the *rated_buffer* to add a rating to these entries.
        """
        self.model.update_model(self.__rated_statistics_buffer_lock, self.__topic_statistics_buffer, self.__host_statistics_buffer, self.__node_statistics_buffer)


    def __add_rated_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: RatedStatistics
        """
        #todo: are the locks necessary here? e.g. can this be called multiple times by the same subscriber?
        self.__rated_statistics_buffer_lock.aquire()
        self.__rated_statistics_buffer.append(item)
        self.__rated_statistics_buffer_lock.release()

    def __add_topic_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: TopicStatistics
        """
        self.__topic_statistics_buffer_lock.aquire()
        self.__topic_statistics_buffer.append(item)
        self.__topic_statistics_buffer_lock.release()


    def __add_node_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: NodeStatistics
        """
        self.__node_statistics_buffer_lock.aquire()
        self.__node_statistics_buffer.append(item)
        self.__node_statistics_buffer_lock.release()


    def __add_host_statistics_item(self, item):
        """
        Adds the item to the buffer list. Will be called whenever data from the topics is available.

        :param item: the item which will be added to the buffer
        :type item: HostStatistics
        """
        self.__host_statistics_buffer_lock.aquire()
        self.__host_statistics_buffer.append(item)
        self.__host_statistics_buffer_lock.release()