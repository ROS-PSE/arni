from abstract_item import AbstractItem

class HostItem(AbstractItem):


    """A HostItem represents a host with all its data"""


    def __init__(self, seuid, parent=None):
        """Initializes the ConnectionItem

        :param list: connection list
        :type list: list
        :param parent: the parent-object
        :type parent: object
        """
        super(HostItem, self).__init__(parent)

        self.__type = "host"

        #add the content
        self.__attributes = ["window_start", "window_stop", "cpu_temp_mean", "cpu_temp_stddev", "cpu_temp_max", "cpu_usage_mean",
                      "cpu_usage_stddev", "cpu_usage_max", "cpu_usage_core_mean", "cpu_usage_core_stddev",
                      "cpu_usage_core_max", "cpu_temp_core", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                      "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                      "gpu_usage_stddev", "gpu_usage_max", "ram_usage_mean", "ram_usage_stddev", "ram_usage_max",
                      "interface_name", "message_frequency_mean", "message_frequency_stddev",
                      "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                      "drive_name", "drive_free_space", "drive_read", "drive_write"]
        for item in self.__attributes:
            self.__add_data_list(item)



    def execute_action(self, action):
        """Sends a signal to stop or restart a host

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass

