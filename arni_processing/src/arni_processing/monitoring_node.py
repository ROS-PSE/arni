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

    def receive_data(self, data):
        """
        Topic callback method.
        Receives data from the topic, sends them to the storage,
        lets them process and finally publishes the comparison result.

        :param data: The data received from the topic.
        """
        try:
            seuid = SEUID(data)
        except TypeError as msg:
            rospy.logerr("received invalid message type:\n%s" % traceback.format_exc())
        try:
            self.__process_data(data, str(seuid))
        except Exception as msg:
            rospy.logerr("an error occured processing the data:\n%s\n%s" % (msg, traceback.format_exc()))

    def __process_data(self, data, identifier):
        """
        Kicks off the processing of the received data.

        :param data: Host or Node Statistics from the HostStatistics, TopicStatistics or NodeStatistics topics.
        :type data: object
        :param identifier: The seuid identifying the received data.
        :type identifier: str
        :return: RatedStatisticsContainer.
        """
        result = self.__specification_handler.compare(data, identifier)
        container = StorageContainer(rospy.Time.now(), identifier, data, result)
        self.__metadata_storage.store(container)
        self.__publish_data(result.to_msg_type())
        return result

    def __publish_data(self, data):
        """
        Publishes data to the RatedStatistics topic.

        :param data: The data to be published
        :type data: RatedStatistics
        """
        try:
            self.__publisher.publish(data)
        except Exception as msg:
            rospy.logerr(msg)

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
                response.rated_host_statistics.append(container.data_rated.to_msg_type())
            if container.identifier[0] == "n":
                response.node_statistics.append(container.data_raw)
                response.rated_node_statistics.append(container.data_rated.to_msg_type())
            if container.identifier[0] == "c":
                response.topic_statistics.append(container.data_raw)
                response.rated_topic_statistics.append(container.data_rated.to_msg_type())
        return response

    def listener(self):
        rospy.Subscriber('/statistics', rosgraph_msgs.msg.TopicStatistics, self.receive_data)
        rospy.Subscriber('/statistics_host', arni_msgs.msg.HostStatistics, self.receive_data)
        rospy.Subscriber('/statistics_node', arni_msgs.msg.NodeStatistics, self.receive_data)
        rospy.Service('~reload_specifications', std_srvs.srv.Empty, self.__specification_handler.reload_specifications)
        rospy.spin()
