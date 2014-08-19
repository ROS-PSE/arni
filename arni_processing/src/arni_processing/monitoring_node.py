import rospy
import rosgraph_msgs
import std_srvs.srv
from std_srvs.srv import Empty
from arni_msgs.msg import HostStatistics, NodeStatistics
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

    def receive_data(self, data):
        """
        Topic callback method.
        Receives data from the topic, sends them to the storage,
        lets them process and finally publishes the comparison result.

        :param data: The data received from the topic.
        """
        seuid = ""
        if hasattr(data, "cpu_temp_mean"):
            seuid = "h" + SEUID_DELIMITER + data.host
        elif hasattr(data, "node_cpu_usage_mean"):
            seuid = "n" + SEUID_DELIMITER + data.node
        elif hasattr(data, "topic"):
            seuid = "c" + SEUID_DELIMITER + data.node_sub\
                    + SEUID_DELIMITER + data.topic\
                    + SEUID_DELIMITER + data.node_pub
        self.__process_data(data, seuid)

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
        container = StorageContainer()
        container.data_raw = data
        container.data_rated = result
        container.timestamp = rospy.Time.now()
        container.identifier = identifier
        self.__metadata_storage.store(container)
        self.__publish_data(result)
        return result

    def __publish_data(self, data):
        """
        Publishes data to the RatedStatistics topic.

        :param data: The data to be published
        :type data: RatedStatisticsContainer
        """
        pass

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
        rospy.Subscriber('/statistics', TopicStatistics, self.receive_data)
        rospy.Subscriber('/statistics_host', HostStatistics, self.receive_data)
        rospy.Subscriber('/statistics_node', NodeStatistics, self.receive_data)
        rospy.Service('~reload_specifications', std_srvs.srv.Empty, self.__specification_handler.reload_specifications)
        rospy.spin()
