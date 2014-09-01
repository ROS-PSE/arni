from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation

class HostItem(AbstractItem):
    """
    A HostItem represents a host with all its data.
    """

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
        self.__parent = parent
        self._type = "host"

        self._attributes = []
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
            
        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new HostItem")


    def execute_action(self, action):
        """
        Not senseful, Host cannot execute actions.

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass


    def get_detailed_data(self):
        """
        Returns the detailed data of the HostItem.
        
        :returns: detailed data
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += self.tr("cpu_usage_mean") + ": " + str(data_dict["cpu_usage_mean"]) \
                   + " " + self.tr("cpu_usage_mean_unit") + " <br>"
        content += self.tr("cpu_usage_stddev") + ": " + str(data_dict["cpu_usage_stddev"]) \
                   + " " + self.tr("cpu_usage_stddev_unit") + " <br>"
        content += self.tr("cpu_usage_max") + ": " + str(data_dict["cpu_usage_max"]) \
                   + " " + self.tr("cpu_usage_max_unit") + " <br>"
        content += self.tr("cpu_temp_mean") + ": " + str(data_dict["cpu_temp_mean"]) \
                   + " " + self.tr("cpu_temp_mean_unit") + " <br>"
        content += self.tr("cpu_temp_stddev") + ": " + str(data_dict["cpu_temp_stddev"]) \
                   + " " + self.tr("cpu_temp_stddev_unit") + " <br>"
        content += self.tr("cpu_temp_max") + ": " + str(data_dict["cpu_temp_max"]) \
                   + " " + self.tr("cpu_temp_max_unit") + " <br>"
        content += self.tr("ram_usage_mean") + ": " + str(data_dict["ram_usage_mean"]) \
                   + " " + self.tr("ram_usage_mean_unit") + " <br>"
        content += self.tr("ram_usage_stddev") + ": " + str(data_dict["ram_usage_stddev"]) \
                   + " " + self.tr("ram_usage_stddev_unit") + " <br>"
        content += self.tr("ram_usage_max") + ": " + str(data_dict["ram_usage_max"]) \
                   + " " + self.tr("ram_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["cpu_usage_core_mean"])):
            content += self.tr("core" + str(i + 1)) + "<br>"
            content += self.tr("cpu_usage_core_mean") + ": " + str(data_dict["cpu_usage_core_mean"][i]) \
                       + " " + self.tr("cpu_usage_core_mean_unit") + " <br>"
            content += self.tr("cpu_usage_core_stddev") + ": " + str(data_dict["cpu_usage_core_stddev"][i]) \
                       + " " + self.tr("cpu_usage_core_stddev_unit") + " <br>"
            content += self.tr("cpu_usage_core_max") + ": " + str(data_dict["cpu_usage_core_max"][i]) \
                       + " " + self.tr("cpu_usage_core_max_unit") + " <br>"
            content += self.tr("cpu_temp_core_mean") + ": " + str(data_dict["cpu_temp_core_mean"][i]) \
                       + " " + self.tr("cpu_temp_core_mean_unit") + " <br>"
            content += self.tr("cpu_temp_core_stddev") + ": " + str(data_dict["cpu_temp_core_stddev"][i]) \
                       + " " + self.tr("cpu_temp_core_stddev_unit") + " <br>"
            content += self.tr("cpu_temp_core_max") + ": " + str(data_dict["cpu_temp_core_max"][i]) \
                       + " " + self.tr("cpu_temp_core_max_unit") + " <br>"

        for i in range(0, len(data_dict["gpu_usage_mean"])):
            content += self.tr("gpu_temp_mean") + ": " + str(data_dict["gpu_temp_mean"][i]) \
                       + " " + self.tr("gpu_temp_mean_unit") + " <br>"
            content += self.tr("gpu_temp_stddev") + ": " + str(data_dict["gpu_temp_stddev"][i]) \
                       + " " + self.tr("gpu_temp_stddev_unit") + " <br>"
            content += self.tr("gpu_temp_max") + ": " + str(data_dict["gpu_temp_max"][i]) \
                       + " " + self.tr("gpu_temp_max_unit") + " <br>"
            content += self.tr("gpu_usage_mean") + ": " + str(data_dict["gpu_usage_mean"][i]) \
                       + " " + self.tr("gpu_usage_mean_unit") + " <br>"
            content += self.tr("gpu_usage_stddev") + ": " + str(data_dict["gpu_usage_stddev"][i]) \
                       + " " + self.tr("gpu_usage_stddev_unit") + " <br>"
            content += self.tr("gpu_usage_max") + ": " + str(data_dict["gpu_usage_max"][i]) \
                       + " " + self.tr("gpu_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["interface_name"])):
            content += str(data_dict["interface_name"][i]) + "<br>"
            content += self.tr("message_frequency_mean") + ": " + str(data_dict["message_frequency_mean"][i]) \
                       + " " + self.tr("message_frequency_mean_unit") + " <br>"
            content += self.tr("message_frequency_stddev") + ": " + str(data_dict["message_frequency_stddev"][i]) \
                       + " " + self.tr("message_frequency_stddev_unit") + " <br>"
            content += self.tr("message_frequency_max") + ": " + str(data_dict["message_frequency_max"][i]) \
                       + " " + self.tr("message_frequency_max_unit") + " <br>"
            content += self.tr("bandwidth_mean") + ": " + str(data_dict["bandwidth_mean"][i]) \
                       + " " + self.tr("bandwidth_mean_unit") + " <br>"
            content += self.tr("bandwidth_stddev") + ": " + str(data_dict["bandwidth_stddev"][i]) \
                       + " " + self.tr("bandwidth_stddev_unit") + " <br>"
            content += self.tr("bandwidth_max") + ": " + str(data_dict["bandwidth_max"][i]) \
                       + " " + self.tr("bandwidth_max_unit") + " <br>"

        for i in range(0, len(data_dict["drive_name"])):
            content += data_dict["drive_name"][i] + " <br>"
            content += self.tr("drive_free_space") + ": " + str(data_dict["drive_free_space"][i]) \
                       + " " + self.tr("drive_free_space_unit") + " <br>"
            content += self.tr("drive_read") + ": " + str(data_dict["drive_read"][i]) \
                       + " " + self.tr("drive_read_unit") + " <br>"
            content += self.tr("drive_write") + ": " + str(data_dict["drive_write"][i]) \
                       + " " + self.tr("drive_write_unit") + " <br>"
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
        """
        Returns a shortend version of the item data.

        :returns: data of the item
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = ""
        if data_dict["state"] is not "ok":
            #todo: print the wrong data here
            content += "something is wrong"
            pass
        else:
            content += self.tr("cpu_usage_mean") + ": " + str(data_dict["cpu_usage_mean"]) \
                   + " " + self.tr("cpu_usage_mean_unit") + " - "
            content += self.tr("cpu_temp_mean") + ": " + str(data_dict["cpu_temp_mean"]) \
                   + " " + self.tr("cpu_temp_mean_unit") + " - "
            content += self.tr("ram_usage_mean") + ": " + str(data_dict["ram_usage_mean"]) \
                   + " " + self.tr("ram_usage_mean_unit")

        return content


    def _get_list_items(self):
        return ["cpu_usage_core_mean", "cpu_usage_core_stddev",
                "cpu_usage_core_max", "cpu_temp_core", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                "gpu_usage_stddev", "gpu_usage_max", "interface_name", "message_frequency_mean",
                "message_frequency_stddev",
                "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                "drive_name", "drive_free_space", "drive_read", "drive_write"]