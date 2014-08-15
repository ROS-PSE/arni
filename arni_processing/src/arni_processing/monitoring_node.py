import rospy
import rosgraph_msgs
from arni_msgs.msg import HostStatistics, NodeStatistics
from rosgraph_msgs.msg import TopicStatistics
from metadata_storage import MetadataStorage
from specification_handler import SpecificationHandler
from rated_statistics import RatedStatistics


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
        # switch type
        self.__process_data(data)

    def __process_data(self, data):
        """
        Kicks off the processing of the received data.

        :param data: Host or Node Statistics from the HostStatistics, TopicStatistics or NodeStatistics topics.
        :return: RatedStatistics.
        """
        result = self.__specification_handler.compare(data)
        return result

    def __publish_data(self, data):
        """
        Publishes data to the RatedStatistics topic.

        :param data: The data to be published
        :type data: RatedStatistics
        """
        pass

    def storage_server(self, request):
        """
        Returns StorageContainer objects on request.

        :param request: The request containing a timestamp and an identifier.
        :type request: MetadataStorageRequest.
        :returns: MetadataStorageResponse
        """
        pass

    def listener(self):
        rospy.Subscriber('/statistics', TopicStatistics, self.receive_data)
        rospy.Subscriber('/statistics_hosts', HostStatistics, self.receive_data)
        rospy.Subscriber('/statistics_nodes', NodeStatistics, self.receive_data)
        rospy.spin()
