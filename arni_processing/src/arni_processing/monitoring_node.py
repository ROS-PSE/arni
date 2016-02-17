import rospy
import traceback
import rosgraph_msgs
import std_srvs.srv
from std_srvs.srv import Empty
import arni_msgs
from arni_msgs.msg import HostStatistics, NodeStatistics, RatedStatistics, RatedStatisticsEntity, MasterApi, MasterApiEntity
from arni_msgs.srv import StatisticHistory, StatisticHistoryResponse
from arni_core.helper import *
from rosgraph_msgs.msg import TopicStatistics
from metadata_storage import MetadataStorage
from specification_handler import SpecificationHandler
from rated_statistics import RatedStatisticsContainer
from storage_container import StorageContainer
import rosgraph

class MonitoringNode:
    """
    Processes and rates incoming topicstatistics,
    stores them for some time and publishes comparison results.
    """

    def __init__(self):
        self.__metadata_storage = MetadataStorage()
        self.__specification_handler = SpecificationHandler()
        self.__publisher = rospy.Publisher('/statistics_rated', arni_msgs.msg.RatedStatistics, queue_size=50)
        self.__master_api_publisher = rospy.Publisher('/statistics_master', arni_msgs.msg.MasterApi, queue_size=10)
        self.__pub_queue = []
        self.__master_api_queue =[]
        self.__aggregate = []
        self.__aggregation_window = rospy.get_param("~aggregation_window", 3)
        self.__aggregate_start = rospy.Time.now()
        self.__processing_enabled = rospy.get_param("/enable_statistics", False)
        self.__alive_timers = {}
        self.__alive_countdown = {}
        rospy.Timer(rospy.Duration(rospy.get_param("~publish_interval", 5)), self.__publish_queue)
        rospy.Timer(rospy.Duration(rospy.get_param("~alive_interval", 5)), self.__check_alive)
        rospy.Timer(rospy.Duration(rospy.get_param("~master_api_publish_interval", 1)), self.__pollMasterAPI)
        rospy.Timer(rospy.Duration(rospy.get_param("/arni/check_enabled_interval", 10)), self.__update_enabled)

    def __pollMasterAPI(self, event):
        """
        Regularly polls the master api to get the most recent system state from rosgraph. This data is then published
        on the /statistics_master topic.
        """
        # poll master api and get most recent data
        self.master = rosgraph.masterapi.Master("")
        state = None
        try:
            state = self.master.getSystemState()
        except rosgraph.masterapi.MasterException as e:
            rospy.logerr("an error occured trying to connect to the master:\n%s\n%s" % (str(e), traceback.format_exc()))
            return

        pubs, subs, srvs = state
        msg = MasterApi()
        ent = MasterApiEntity()
        for (topic, publisher) in pubs:
            ent.name = topic
            ent.content = publisher
            msg.pubs.append(ent)
        for (topic, subscriber) in subs:
            ent.name = topic
            ent.content = subscriber
            msg.subs.append(ent)
        for (service, provider) in srvs:
            ent.name = service
            ent.content = provider
            msg.srvs.append(ent)

        # publish this data on /statistics_master
        self.__master_api_publisher.publish(msg)

    def __update_enabled(self, event):
        self.__processing_enabled = rospy.get_param("/enable_statistics", False)

    def receive_master_api_data(self, data):
        """
        Topic callback for incoming master api messages.
        """
        # pubs, subs, srvs = data.pubs, data.subs, data.srvs

        seuids = generate_seuids_from_master_api_data(data)

        for seuid in seuids:
            # for now only report that this connection is still alive - even though it might no longer transport
            # any data
            # TODO does this change the current behaviour of the program? And how could I prevent it
            self.__report_alive(seuid)

    def receive_data(self, data):
        """
        Topic callback method.
        Receives data from the topic, sends them to the storage,
        lets them process and finally publishes the comparison result.

        :param data: The data received from the topic.
        """
        if self.__processing_enabled:
            try:
                seuid = SEUID(data)
                self.__report_alive(str(seuid))
                if seuid.topic is not None:
                    self.__report_alive(str(seuid.get_seuid("topic")))
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

    def __check_alive(self, event):
        """
        Iterates over all registered specifications and sends an error if no package is received but was expected.
        """
        for seuid in self.__specification_handler.loaded_specifications():
            if seuid not in self.__alive_timers.keys():
                spec = self.__specification_handler.get(seuid)
                alive_timer = spec.get("alive_timer")
                if not alive_timer:
                    alive_timer = rospy.get_param("~alive_timer", 10)
                else:
                    alive_timer = alive_timer[1]
                self.__alive_timers[seuid] = rospy.Duration(alive_timer)
            if seuid not in self.__alive_countdown.keys():
                self.__alive_countdown[seuid] = rospy.Time.now()
            if rospy.Time.now() >= self.__alive_countdown[seuid] + self.__alive_timers[seuid]:
                r = RatedStatisticsContainer(seuid)
                r.add_value("alive", ["False"], ["True"], [1])
                r.add_value("window_start", self.__alive_countdown[seuid], None, None)
                r.add_value("window_stop", rospy.Time.now(), None, None)
                msg = r.to_msg_type()
                self.__publish_data(msg)

    def __report_alive(self, seuid):
        """
        Report that messages with the given seuid still arrive.

        :param seuid: The seuid of the message that arrived.
        """
        self.__alive_countdown[seuid] = rospy.Time.now()

    def __aggregate_data(self, data, identifier):
        """
        Collect topic data and send them to get rated after a while.

        :param data: A statistics message object
        """
        res = self.__specification_handler.compare_topic(self.__aggregate)
        if self.__aggregate is None or \
                older_than(self.__aggregate_start, rospy.Duration(self.__aggregation_window)):
            for r in res:
                container = StorageContainer(rospy.Time.now(), str(identifier), data, r)
                self.__metadata_storage.store(container)
                self.__publish_data(r, False)
            current_data = []
            for message in self.__aggregate:
              if not older_than(message.window_stop, rospy.Duration(self.__aggregation_window)):
                current_data.append(message)
            self.__aggregate = current_data
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
        :type request: StatisticHistoryRequest.
        :returns: StatisticHistoryResponse
        """
        data = self.__metadata_storage.get("*", request.timestamp)
        response = StatisticHistoryResponse()
        for container in data:
            if container.identifier[0] == "h":
                if rospy.Time.now() >= self.__alive_countdown[container.identifier] + self.__alive_timers[container.identifier]:
                    print("no longer alive - not reporting.")
                else:
                    response.host_statistics.append(container.data_raw)
                    response.rated_host_statistics.append(container.data_rated)
            elif container.identifier[0] == "n":
                if rospy.Time.now() >= self.__alive_countdown[container.identifier] + self.__alive_timers[container.identifier]:
                    print("no longer alive - not reporting.")
                else:
                    response.node_statistics.append(container.data_raw)
                    response.rated_node_statistics.append(container.data_rated)
            elif container.identifier[0] == "c":
                if rospy.Time.now() >= self.__alive_countdown[container.identifier] + self.__alive_timers[container.identifier]:
                    print("no longer alive - not reporting.")
                else:
                    response.topic_statistics.append(container.data_raw)
                    response.rated_topic_statistics.append(container.data_rated)
            elif container.identifier[0] == "t":
                if rospy.Time.now() >= self.__alive_countdown[container.identifier] + self.__alive_timers[container.identifier]:
                    print("no longer alive - not reporting.")
                else:
                    response.rated_topic_statistics.append(container.data_rated)
        return response

    def listener(self):
        """
        Sets up all necessary subscribers and services.
        """
        rospy.Subscriber('/statistics', rosgraph_msgs.msg.TopicStatistics, self.receive_data)
        rospy.Subscriber('/statistics_host', arni_msgs.msg.HostStatistics, self.receive_data)
        rospy.Subscriber('/statistics_node', arni_msgs.msg.NodeStatistics, self.receive_data)
        rospy.Subscriber('/statistics_master', arni_msgs.msg.MasterApi, self.receive_master_api_data)
        rospy.Service('~reload_specifications', std_srvs.srv.Empty, self.__specification_handler.reload_specifications)
        rospy.Service('~get_statistic_history', arni_msgs.srv.StatisticHistory, self.storage_server)
        rospy.spin()
