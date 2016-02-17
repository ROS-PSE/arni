from rospy.rostime import Time, Duration
from rospy import ServiceException
import rospy

from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from arni_core.host_lookup import HostLookup
from arni_core.helper import SEUID
import arni_core.helper as helper
from arni_msgs.srv import NodeReaction
from helper_functions import prepare_number_for_representation, MAXIMUM_OFFLINE_TIME, ROUND_DIGITS


class NodeItem(AbstractItem):
    """
    A NodeItem represents a node with all of its data. It also has a interface to start/stop/restart nodes.
    """

    def __init__(self, logger, seuid, parent=None):
        """
        Initializes the NodeItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        self._type = "node"
        self.__parent = parent

        self._type = "node"

        self._attributes = []
        self._attributes.extend(["node_cpu_usage_mean", "node_cpu_usage_stddev", "node_cpu_usage_max",
                                 "node_cpu_usage_core_mean",
                                 "node_cpu_usage_core_stddev", "node_cpu_usage_core_max", "node_gpu_usage_mean",
                                 "node_gpu_usage_stddev",
                                 "node_gpu_usage_max", "node_ramusage_mean", "node_ramusage_stddev",
                                 "node_ramusage_max",
                                 "node_message_frequency_mean", "node_message_frequency_stddev",
                                 "node_message_frequency_max", "node_bandwidth_mean", "node_bandwidth_stddev",
                                 "node_bandwidth_max", "node_write_mean", "node_write_stddev", "node_write_max",
                                 "node_read_mean",
                                 "node_read_stddev", "node_read_max"])

        for item in self._attributes:
            self._add_data_list(item)

        for item in self._attributes:
            self._rated_attributes.append(item + ".actual_value")
            self._rated_attributes.append(item + ".expected_value")
            self._rated_attributes.append(item + ".state")

        for item in self._rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new NodeItem")

    def execute_action(self, action):
        """
        Sends a signal to top or restart the node.

        :param action: action to be executed
        :type action: RemoteAction
        """
        host_formatted = helper.underscore_ip(self.__parent.get_seuid()[2:])
        service_name = "/execute_node_reaction/%s" % host_formatted
        try:
            execute = rospy.ServiceProxy(
                service_name, NodeReaction)
            resp = execute(self.seuid[2:], action, '')
        except ServiceException:
            self._logger.log("error", Time.now(), self.seuid, "could not stop node %s, service %s not found"
                             % (self.seuid, service_name))


    def get_detailed_data(self):
        """
        Returns the detailed data of the NodeItem.
        
        :returns: detailed data
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += self.tr("node_cpu_usage_mean") + ": " + prepare_number_for_representation(
            data_dict["node_cpu_usage_mean"]) + " " + self.tr("node_cpu_usage_mean_unit") + " <br>"
        content += self.tr("node_cpu_usage_stddev") + ": " + prepare_number_for_representation(
            data_dict["node_cpu_usage_stddev"]) \
                   + " " + self.tr("node_cpu_usage_stddev_unit") + " <br>"
        content += self.tr("node_cpu_usage_max") + ": " + prepare_number_for_representation(
            data_dict["node_cpu_usage_max"]) \
                   + " " + self.tr("node_cpu_usage_max_unit") + " <br>"
        content += self.tr("node_ramusage_mean") + ": " + prepare_number_for_representation(
            data_dict["node_ramusage_mean"]) \
                   + " " + self.tr("node_ramusage_mean_unit") + " <br>"
        content += self.tr("node_ramusage_stddev") + ": " + prepare_number_for_representation(
            data_dict["node_ramusage_stddev"]) \
                   + " " + self.tr("node_ramusage_stddev_unit") + " <br>"
        content += self.tr("node_ramusage_max") + ": " + prepare_number_for_representation(
            data_dict["node_ramusage_max"]) \
                   + " " + self.tr("node_ramusage_max_unit") + " <br>"

        for i in range(0, len(data_dict["node_cpu_usage_core_mean"])):
            content += self.tr("core" + str(i + 1)) + "<br>"
            content += self.tr("node_cpu_usage_core_mean") + ": " + prepare_number_for_representation(
                data_dict["node_cpu_usage_core_mean"][i]) \
                       + " " + self.tr("node_cpu_usage_core_mean_unit") + " <br>"
            content += self.tr("node_cpu_usage_core_stddev") + ": " + prepare_number_for_representation(
                data_dict["node_cpu_usage_core_stddev"][i]) \
                       + " " + self.tr("node_cpu_usage_core_stddev_unit") + " <br>"
            content += self.tr("node_cpu_usage_core_max") + ": " + prepare_number_for_representation(
                data_dict["node_cpu_usage_core_max"][i]) \
                       + " " + self.tr("node_cpu_usage_core_max_unit") + " <br>"

        for i in range(0, len(data_dict["node_gpu_usage_mean"])):
            content += self.tr("node_gpu_usage_mean") + ": " + prepare_number_for_representation(
                data_dict["node_gpu_usage_mean"][i]) \
                       + " " + self.tr("node_gpu_usage_mean_unit") + " <br>"
            content += self.tr("node_gpu_usage_stddev") + ": " + prepare_number_for_representation(
                data_dict["node_gpu_usage_stddev"][i]) \
                       + " " + self.tr("node_gpu_usage_stddev_unit") + " <br>"
            content += self.tr("node_gpu_usage_max") + ": " + prepare_number_for_representation(
                data_dict["node_gpu_usage_max"][i]) \
                       + " " + self.tr("node_gpu_usage_max_unit") + " <br>"


        content += self.tr("node_message_frequency_mean") + ": " + prepare_number_for_representation(
            data_dict["node_message_frequency_mean"]) \
                   + " " + self.tr("node_message_frequency_mean_unit") + " <br>"
        content += self.tr("node_message_frequency_stddev") + ": " + prepare_number_for_representation(
            data_dict["node_message_frequency_stddev"]) \
                   + " " + self.tr("node_message_frequency_stddev_unit") + " <br>"
        content += self.tr("node_message_frequency_max") + ": " + prepare_number_for_representation(
            data_dict["node_message_frequency_max"]) \
                   + " " + self.tr("node_message_frequency_max_unit") + " <br>"
        content += self.tr("node_bandwidth_mean") + ": " + prepare_number_for_representation(
            data_dict["node_bandwidth_mean"]) \
                   + " " + self.tr("node_bandwidth_mean_unit") + " <br>"
        content += self.tr("node_bandwidth_stddev") + ": " + prepare_number_for_representation(
            data_dict["node_bandwidth_stddev"]) \
                   + " " + self.tr("node_bandwidth_stddev_unit") + " <br>"
        content += self.tr("node_bandwidth_max") + ": " + prepare_number_for_representation(
            data_dict["node_bandwidth_max"]) \
                   + " " + self.tr("node_bandwidth_max_unit") + " <br>"
        content += self.tr("node_write_mean") + ": " + prepare_number_for_representation(data_dict["node_write_mean"]) \
                   + " " + self.tr("node_write_mean_unit") + " <br>"
        content += self.tr("node_write_stddev") + ": " + prepare_number_for_representation(
            data_dict["node_write_stddev"]) \
                   + " " + self.tr("node_write_stddev_unit") + " <br>"
        content += self.tr("node_write_max") + ": " + prepare_number_for_representation(data_dict["node_write_max"]) \
                   + " " + self.tr("node_write_max_unit") + " <br>"
        content += self.tr("node_read_mean") + ": " + prepare_number_for_representation(data_dict["node_read_mean"]) \
                   + " " + self.tr("node_read_mean_unit") + " <br>"
        content += self.tr("node_read_stddev") + ": " + prepare_number_for_representation(data_dict["node_read_stddev"]) \
                   + " " + self.tr("node_read_stddev_unit") + " <br>"
        content += self.tr("node_read_max") + ": " + prepare_number_for_representation(data_dict["node_read_max"]) \
                   + " " + self.tr("node_read_max_unit") + " <br>"
        content += "</p>"

        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["node_cpu_usage_mean", "node_cpu_usage_stddev", "node_cpu_usage_max",
                                 "node_cpu_usage_core_mean",
                                 "node_cpu_usage_core_stddev", "node_cpu_usage_core_max", "node_gpu_usage_mean",
                                 "node_gpu_usage_stddev",
                                 "node_gpu_usage_max", "node_ramusage_mean", "node_ramusage_stddev",
                                 "node_ramusage_max",
                                 "node_message_frequency_mean", "node_message_frequency_stddev",
                                 "node_message_frequency_max", "node_bandwidth_mean", "node_bandwidth_stddev",
                                 "node_bandwidth_max", "node_write_mean", "node_write_stddev", "node_write_max",
                                 "node_read_mean",
                                 "node_read_stddev", "node_read_max"]

    def get_short_data(self):
        """
        Returns a shortend version of the item data.
        
        :returns: data of the item
        :rtype: str
        """
        data_dict = self.get_latest_data()

        if data_dict["window_stop"] == Time(0):
            return "No data yet"
        elif (Time.now() - data_dict["window_stop"]) > Duration(MAXIMUM_OFFLINE_TIME):
            # last entry was more than MAXIMUM_OFFLINE_TIME ago, it could be offline!
            return "No data since " + str(round((Time.now() - data_dict["window_stop"]).to_sec(), ROUND_DIGITS)) \
                   + " seconds"

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries_for_log()
        else:
            content += self.tr("node_cpu_usage_mean") + ": " + prepare_number_for_representation(
            data_dict["node_cpu_usage_mean"]) + " " + self.tr("node_cpu_usage_mean_unit") + " - "
            content += self.tr("node_ramusage_mean") + ": " + prepare_number_for_representation(
            data_dict["node_ramusage_mean"]) \
                   + " " + self.tr("node_ramusage_mean_unit") + " - "
            content += self.tr("node_message_frequency_mean") + ": " + prepare_number_for_representation(
            data_dict["node_message_frequency_mean"]) \
                   + " " + self.tr("node_message_frequency_mean_unit") + " - "
            content += self.tr("node_bandwidth_mean") + ": " + prepare_number_for_representation(
            data_dict["node_bandwidth_mean"]) \
                   + " " + self.tr("node_bandwidth_mean_unit")

        return content

    def can_execute_actions(self):
        """
        This item can execute actions, so it returns True

        :return: True
        """
        return True

    def get_list_items(self):
        return ["node_cpu_usage_core_mean", "node_cpu_usage_core_stddev", "node_cpu_usage_core_max",
                "node_gpu_usage_mean", "node_gpu_usage_stddev", "node_gpu_usage_max"]

    def get_time_items(self):
        return []
