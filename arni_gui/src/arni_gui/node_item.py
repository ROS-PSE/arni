from abstract_item import AbstractItem
from python_qt_binding.QtCore import QObject

from arni_core.host_lookup import HostLookup
from arni_core.helper import SEUID
import arni_core.helper as helper
from arni_msgs.srv import NodeReaction

import rospy
from rospy import ServiceException



class NodeItem(AbstractItem):
    """A NodeItem represents a node with all of its data. It also has a interface to start/stop/restart nodes."""


    def __init__(self, seuid, parent=None):
        """
        Initializes the NodeItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        #add the content
        AbstractItem.__init__(self, seuid, parent)
        #super(NodeItem, self).__init__(seuid, parent)
        self._type = "node"
        self.__parent = parent

        self._type = "node"

        self._attributes = []
        self._attributes.extend(["node_cpu_usage_mean", "node_cpu_usage_stddev", "node_cpu_usage_max",
                      "node_cpu_usage_core_mean",
                      "node_cpu_usage_core_stddev", "node_cpu_usage_core_max", "node_gpu_usage_mean", "node_gpu_usage_stddev",
                      "node_gpu_usage_max", "node_ramusage_mean", "node_ramusage_stddev", "node_ramusage_max",
                      "node_message_frequency_mean", "node_message_frequency_stddev", "node_message_frequency_max", "node_bandwidth_mean", "node_bandwidth_stddev",
                      "node_bandwidth_max", "node_write_mean", "node_write_stddev", "node_write_max", "node_read_mean",
                      "node_read_stddev", "node_read_max"])

        for item in self._attributes:
            self._add_data_list(item)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)


    def execute_action(self, action):
        """
        Sends a signal to top or restart the node.

        :param action: action to be executed
        :type action: RemoteAction
        """
        #create the name of the service
        #service_name = "/execute_node_reaction/" +
        #rospy.wait_for_service(
        
        host_formatted = helper.underscore_ip(self.__parent.get_seuid()[2:])
        service_name = "/execute_node_reaction/%s" % host_formatted
        try:            
            execute = rospy.ServiceProxy(
                service_name, NodeReaction)
            resp = execute(self.seuid[2:], action, '')
            #rospy.logdebug(
                #"sending command '%s' to node %s returned: %s"
                #% (self.__command, self._node, resp.returnmessage))
            print resp.returnmessage
        except ServiceException:
            rospy.logdebug(
                "could not stop node %s, service %s not found"
                % (self.seuid, service_name))

    def get_detailed_data(self):
        """
        Returns the detailed data of the NodeItem.
        
        :returns: str
        """
        #todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p style=\"font-size:15px\">"

        content += "node_cpu_usage_mean: " + str(data_dict["node_cpu_usage_mean"]) + "<br>"
        content += "node_bandwidth_mean: " + str(data_dict["node_bandwidth_mean"]) + "<br>"
        # content += "connected_nodes:" + str(data_dict["connected_nodes"]) + "<br>"
        # content += "topic_counter" + str(data_dict["topic_counter"]) + "<br>"
        # content += "connection_counter: " + str(data_dict["connection_counter"]) + "<br>"
        # content += "cpu_usage_max: " + str(data_dict["cpu_usage_max"]) + "<br>"
        # content += "cpu_temp_mean: " + str(data_dict["cpu_temp_mean"]) + "<br>"
        # content += "average_ram_load: " + str(data_dict["average_ram_load"]) + "<br>"
        # content += "cpu_usage_mean:" + str(data_dict["cpu_usage_mean"]) + "<br>"
        # content += "cpu_temp_max: " + str(data_dict["cpu_temp_max"]) + "<br>"
        # content += "ram_usage_max: " + str(data_dict["ram_usage_max"]) + "<br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["node_cpu_usage_mean", "node_bandwidth_mean"]

    def get_short_data(self):
        return "NodeItem"
