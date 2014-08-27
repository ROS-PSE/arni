import rospy
import traceback
import rosgraph_msgs
import std_srvs.srv
from std_srvs.srv import Empty
import arni_msgs
from arni_msgs.msg import HostStatistics, NodeStatistics, RatedStatistics
from arni_msgs.srv import StatisticHistory
from arni_core.helper import *
from rosgraph_msgs.msg import TopicStatistics
from metadata_storage import MetadataStorage
from specification_handler import SpecificationHandler
from rated_statistics import RatedStatisticsContainer
from storage_container import StorageContainer


class MonitoringNode:
    """
    Processes and rates incoming topicstatistics,
    stores them for some time and publishes comparison results.
    """

    def __init__(self):
        self.__metadata_storage = MetadataStorage()
        self.__specification_handler = SpecificationHandler()
        self.__publisher = rospy.Publisher('/statistics_rated', arni_msgs.msg.RatedStatistics, queue_size=50)
        self.__pub_queue = []
        self.__aggregate = []
        self.__aggregate_start = rospy.Time.now()
        rospy.Timer(rospy.Duration(5), self.__publish_data)

    def receive_data(self, data):
        """
        Topic callback method.
        Receives data from the topic, sends them to the storage,
        lets them process and finally publishes the comparison result.

        :param data: The data received from the topic.
        """
        try:
            seuid = SEUID(data)
            try:
                self.__process_data(data, seuid)
            except Exception as msg:
                rospy.logerr("an error occured processing the data:\n%s\n%s" % (msg, traceback.format_exc()))
        except TypeError as msg:
            rospy.logerr("received invalid message type:\n%s\n%s" % (msg, traceback.format_exc()))
        except NameError as msg:
            rospy.logerr("received invalid message type (%s):\n%s\n%s" % (type(data), msg, traceback.format_exc()))

    def __process_data(self, data, identifier):
        """
        Kicks off the processing of the received data.

        :param data: Host or Node Statistics from the HostStatistics, TopicStatistics or NodeStatistics topics.
        :type data: object
        :param identifier: The seuid identifying the received data.
        :type identifier: str
        :return: RatedStatisticsContainer.
        """
        if str(identifier)[0] == "c":
            self.__aggregate_data(data, identifier)
        result = self.__specification_handler.compare(data, str(identifier)).to_msg_type()
        container = StorageContainer(rospy.Time.now(), str(identifier), data, result)
        self.__metadata_storage.store(container)
        self.__publish_data(result, False)
        return result

    def __aggregate_data(self, data, identifier):
        """
        Collect topic data and send them to get rated after a while.

        :param data: A statistics message object
        """
        if self.__aggregate is None or \
                                rospy.Time.now() - self.__aggregate_start >= \
                        rospy.Duration(rospy.get_param("/arni/aggregation_window", 3)):
            res = self.__specification_handler.compare_topic(self.__aggregate)
            for r in res:
                container = StorageContainer(rospy.Time.now(), str(identifier), data, r)
                self.__metadata_storage.store(container)
                self.__publish_data(r, False)
            self.__aggregate = []
            self.__aggregate_start = rospy.Time.now()
        self.__aggregate.append(data)

    def __publish_data(self, data, queue=True):
        """
        Pushes a RatedStatistics object to the queue to publish.

        :param data: RatedStatistics object
        """
        if queue:
            self.__pub_queue.append(data)
        else:
            self.__publisher.publish(data)

    def __publish_queue(self, event):
        """
        Publishes data to the RatedStatistics topic.

        :param event: rospy.TimerEvent
        """
        for data in self.__pub_queue:
            self.__publisher.publish(data)
        self.__pub_queue = []

    def storage_server(self, request):
        """
        Returns StorageContainer objects on request.

        :param request: The request containing a timestamp and an identifier.
        :type request: MetadataStorageRequest.
        :returns: MetadataStorageResponse
        """
        data = self.__metadata_storage.get("*", request.timestamp)
        response = StatisticHistory()
        for container in data:
            if container.identifier[0] == "h":
                response.host_statistics.append(container.data_raw)
                response.rated_host_statistics.append(container.data_rated)
            if container.identifier[0] == "n":
                response.node_statistics.append(container.data_raw)
                response.rated_node_statistics.append(container.data_rated)
            if container.identifier[0] == "c":
                response.topic_statistics.append(container.data_raw)
                response.rated_topic_statistics.append(container.data_rated)
            if container.identifier[0] == "t":
                response.rated_topic_statistics.append(container.data_rated)
        return response

    def listener(self):
        """
        Sets up all necessary subscribers and services.
        """
        rospy.Subscriber('/statistics', rosgraph_msgs.msg.TopicStatistics, self.receive_data)
        rospy.Subscriber('/statistics_host', arni_msgs.msg.HostStatistics, self.receive_data)
        rospy.Subscriber('/statistics_node', arni_msgs.msg.NodeStatistics, self.receive_data)
        rospy.Service('~reload_specifications', std_srvs.srv.Empty, self.__specification_handler.reload_specifications)
        rospy.spin()
