from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem

class HostItem(AbstractItem):
    """A HostItem represents a host with all its data."""

    def __init__(self, logger, seuid, parent=None):
        """
        Initializes the ConnectionItem.

        :param seuid: the seuid of the HostItem
        :type list: list
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        #super(HostItem, self).__init__(seuid, parent)
        self.__parent = parent
        self._type = "host"

        self._attributes = []
        # add the content
        self._attributes.extend(["cpu_temp_mean", "cpu_temp_stddev", "cpu_temp_max", "cpu_usage_mean",
                      "cpu_usage_stddev", "cpu_usage_max", "cpu_usage_core_mean", "cpu_usage_core_stddev",
                      "cpu_usage_core_max", "cpu_temp_core", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                      "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                      "gpu_usage_stddev", "gpu_usage_max", "ram_usage_mean", "ram_usage_stddev", "ram_usage_max",
                      "interface_name", "message_frequency_mean", "message_frequency_stddev",
                      "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                      "drive_name", "drive_free_space", "drive_read", "drive_write"])

        for item in self._attributes:
            self._add_data_list(item)

        #self._attributes.remove("traffic")
        #self._attributes.append("bandwidth")

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

       # del self._attributes

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new HostItem")


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

        content = "<p class=\"detailed_data\">"

        content += QTranslator.translate("AbstractItem", "cpu_temp_stddev") + ": " + str(data_dict["cpu_temp_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "cpu_temp_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "cpu_temp_max") + ": " + str(data_dict["cpu_temp_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "cpu_temp_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "cpu_usage_mean") + ": " + str(data_dict["cpu_usage_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "cpu_usage_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "cpu_usage_stddev") + ": " + str(data_dict["cpu_usage_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "cpu_usage_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "cpu_usage_max") + ": " + str(data_dict["cpu_usage_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "cpu_usage_max_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "ram_usage_mean") + ": " + str(data_dict["ram_usage_mean"]) \
                   + " " + QTranslator.translate("AbstractItem", "ram_usage_mean_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "ram_usage_stddev") + ": " + str(data_dict["ram_usage_stddev"]) \
                   + " " + QTranslator.translate("AbstractItem", "ram_usage_stddev_unit") + " <br>"
        content += QTranslator.translate("AbstractItem", "ram_usage_max") + ": " + str(data_dict["ram_usage_max"]) \
                   + " " + QTranslator.translate("AbstractItem", "ram_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["cpu_usage_core_mean"])):
            content += QTranslator.translate("AbstractItem", "core" + str(i + 1)) + "<br>"
            content += QTranslator.translate("AbstractItem", "cpu_usage_core_mean") + ": " + str(data_dict["cpu_usage_core_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_usage_core_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "cpu_usage_core_stddev") + ": " + str(data_dict["cpu_usage_core_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_usage_core_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "cpu_usage_core_max") + ": " + str(data_dict["cpu_usage_core_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_usage_core_max_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "cpu_temp_core_mean") + ": " + str(data_dict["cpu_temp_core_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_temp_core_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "cpu_temp_core_stddev") + ": " + str(data_dict["cpu_temp_core_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_temp_core_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "cpu_temp_core_max") + ": " + str(data_dict["cpu_temp_core_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "cpu_temp_core_max_unit") + " <br>"

        for i in range(0, len(data_dict["gpu_usage_mean"])):
            content += QTranslator.translate("AbstractItem", "gpu_temp_mean") + ": " + str(data_dict["gpu_temp_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_temp_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "gpu_temp_stddev") + ": " + str(data_dict["gpu_temp_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_temp_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "gpu_temp_max") + ": " + str(data_dict["gpu_temp_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_temp_max_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "gpu_usage_mean") + ": " + str(data_dict["gpu_usage_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_usage_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "gpu_usage_stddev") + ": " + str(data_dict["gpu_usage_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_usage_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "gpu_usage_max") + ": " + str(data_dict["gpu_usage_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "gpu_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["interface_name"])):
            content += str(data_dict["interface_name"]) + "<br>"
            content += QTranslator.translate("AbstractItem", "message_frequency_mean") + ": " + str(data_dict["message_frequency_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "message_frequency_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "message_frequency_stddev") + ": " + str(data_dict["message_frequency_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "message_frequency_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "message_frequency_max") + ": " + str(data_dict["message_frequency_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "message_frequency_max_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "bandwidth_mean") + ": " + str(data_dict["bandwidth_mean"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "bandwidth_mean_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "bandwidth_stddev") + ": " + str(data_dict["bandwidth_stddev"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "bandwidth_stddev_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "bandwidth_max") + ": " + str(data_dict["bandwidth_max"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "bandwidth_max_unit") + " <br>"

        for i in range(0, len(data_dict["drive_name"])):
            content += data_dict["drive_name"] + " <br>"
            content += QTranslator.translate("AbstractItem", "drive_free_space") + ": " + str(data_dict["drive_free_space"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "drive_free_space_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "drive_read") + ": " + str(data_dict["drive_read"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "drive_read_unit") + " <br>"
            content += QTranslator.translate("AbstractItem", "drive_write") + ": " + str(data_dict["drive_write"][i]) \
                       + " " + QTranslator.translate("AbstractItem", "drive_write_unit") + " <br>"

        content += "</p>"
        return content

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["cpu_temp_mean", "cpu_temp_stddev", "cpu_temp_max", "cpu_usage_mean",
                      "cpu_usage_stddev", "cpu_usage_max", "cpu_usage_core_mean", "cpu_usage_core_stddev",
                      "cpu_usage_core_max", "cpu_temp_core", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                      "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                      "gpu_usage_stddev", "gpu_usage_max", "ram_usage_mean", "ram_usage_stddev", "ram_usage_max",
                      "interface_name", "message_frequency_mean", "message_frequency_stddev",
                      "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                      "drive_name", "drive_free_space", "drive_read", "drive_write"]

    def get_short_data(self):
        return "HostItem"