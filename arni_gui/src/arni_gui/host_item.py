from abstract_item import AbstractItem

class HostItem(AbstractItem):
    """A HostItem represents a host with all its data."""


    def __init__(self, seuid, parent=None):
        """
        Initializes the ConnectionItem.

        :param seuid: the seuid of the HostItem
        :type list: list
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        super(HostItem, self).__init__(seuid, parent)
        self.__parent = parent
        self._type = "host"

        #add the content
        self._attributes.extend(["cpu_temp_mean", "cpu_temp_stddev", "cpu_temp_max", "cpu_usage_mean",
                      "cpu_usage_stddev", "cpu_usage_max", "cpu_usage_core_mean", "cpu_usage_core_stddev",
                      "cpu_usage_core_max", "cpu_temp_core", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                      "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                      "gpu_usage_stddev", "gpu_usage_max", "ram_usage_mean", "ram_usage_stddev", "ram_usage_max",
                      "interface_name", "message_frequency_mean", "message_frequency_stddev",
                      "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                      "drive_name", "drive_free_space", "drive_read", "drive_write"])




        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self._attributes:
            self._add_data_list(item)

        del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)


    def execute_action(self, action):
        """
        Sends a signal to stop or restart a host.

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass

    def get_detailed_data(self):
        """
        Returns the detailed data of the HostItem.
        
        :returns: str
        """
        #todo: fill the content sensefully!
        data_dict = self.get_latest_data()

        content = "<p style=\"font-size:15px\">"

        content += "cpu_temp_mean: " + str(data_dict["cpu_temp_mean"]) + "<br>"
        content += "bandwidth_mean: " + str(data_dict["bandwidth_mean"]) + "<br>"
        content += "cpu_temp_stddev:" + str(data_dict["cpu_temp_stddev"]) + "<br>"
        content += "cpu_temp_max" + str(data_dict["cpu_temp_max"]) + "<br>"
        content += "cpu_usage_mean: " + str(data_dict["cpu_usage_mean"]) + "<br>"
        content += "cpu_usage_stddev: " + str(data_dict["cpu_usage_stddev"]) + "<br>"
        content += "cpu_usage_max: " + str(data_dict["cpu_usage_max"]) + "<br>"
        content += "cpu_usage_core_mean: " + str(data_dict["cpu_usage_core_mean"]) + "<br>"
        content += "cpu_usage_core_stddev:" + str(data_dict["cpu_usage_core_stddev"]) + "<br>"
        content += "cpu_temp_core_max: " + str(data_dict["cpu_temp_core_max"]) + "<br>"
        content += "gpu_temp_mean: " + str(data_dict["gpu_temp_mean"]) + "<br>"
        content += "gpu_temp_stddev: " + str(data_dict["gpu_temp_stddev"]) + "<br>"
        content += "gpu_temp_max: " + str(data_dict["gpu_temp_max"]) + "<br>"
        content += "gpu_usage_mean:" + str(data_dict["gpu_usage_mean"]) + "<br>"
        content += "gpu_usage_stddev" + str(data_dict["gpu_usage_stddev"]) + "<br>"
        content += "gpu_usage_max: " + str(data_dict["gpu_usage_max"]) + "<br>"

        content += "</p>"
        return content

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["cpu_temp_mean", "bandwidth_mean"]

    def get_short_data(self):
        return "HostItem"
