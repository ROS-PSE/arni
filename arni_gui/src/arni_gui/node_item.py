from abstract_item import AbstractItem

class NodeItem(AbstractItem):


    """A TopicItem represents a node with all of its data. It also has a interface to start/stop/restart nodes."""


    def __init__(self, seuid, parent=None):
        """Initializes the ConnectionItem

        :param list: connection list
        :type list: list
        :param parent: the parent-object
        :type parent: object
        """
        self.__type = "node"

        AbstractItem.__init__(self, seuid, parent)
        #add the content
        self.__attributes = ["window_start", "window_stop", "node_cpu_usage_mean", "node_cpu_usage_stddev", "node_cpu_usage_max",
                      "node_cpu_usage_core_mean",
                      "node_cpu_usage_core_stddev", "node_cpu_usage_core_max", "node_gpu_usage_mean", "node_gpu_usage_stddev",
                      "node_gpu_usage_max", "node_ramusage_mean", "node_ramusage_stddev", "node_ramusage_max",
                      "node_message_frequency_mean", "node_message_frequency_stddev", "node_message_frequency_max", "node_bandwidth_mean", "node_bandwidth_stddev",
                      "node_bandwidth_max", "node_write_mean", "node_write_stddev", "node_write_max", "node_read_mean",
                      "node_read_stddev", "node_read_max"]
        for item in self.__attributes:
            self.__add_data_list(item)


    def execute_action(self, action):
        """Sends a signal to top or restart the node.

        :param action: action to be executed
        :type action: RemoteAction
        """
        raise NotImplementedError()

