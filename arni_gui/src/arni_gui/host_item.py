from rospy.rostime import Time

from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation, MAXIMUM_OFFLINE_TIME, ROUND_DIGITS
from rospy import Duration


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
                                 "cpu_usage_core_max", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                                 "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max",
                                 "gpu_usage_mean",
                                 "gpu_usage_stddev", "gpu_usage_max", "ram_usage_mean", "ram_usage_stddev",
                                 "ram_usage_max",
                                 "interface_name", "message_frequency_mean", "message_frequency_stddev",
                                 "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                                 "drive_name", "drive_free_space", "drive_read", "drive_write"])

        for item in self._attributes:
            self._add_data_list(item)
       
        for item in self._attributes:
            self._rated_attributes.append(item + ".actual_value")
            self._rated_attributes.append(item + ".expected_value")
            self._rated_attributes.append(item + ".state")

        for item in self._rated_attributes:
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

        content += self.tr("cpu_usage_mean") + ": " + prepare_number_for_representation(data_dict["cpu_usage_mean"]) \
                   + " " + self.tr("cpu_usage_mean_unit") + " <br>"
        content += self.tr("cpu_usage_stddev") + ": " + prepare_number_for_representation(data_dict["cpu_usage_stddev"]) \
                   + " " + self.tr("cpu_usage_stddev_unit") + " <br>"
        content += self.tr("cpu_usage_max") + ": " + prepare_number_for_representation(data_dict["cpu_usage_max"]) \
                   + " " + self.tr("cpu_usage_max_unit") + " <br>"
        content += self.tr("cpu_temp_mean") + ": " + prepare_number_for_representation(data_dict["cpu_temp_mean"]) \
                   + " " + self.tr("cpu_temp_mean_unit") + " <br>"
        content += self.tr("cpu_temp_stddev") + ": " + prepare_number_for_representation(data_dict["cpu_temp_stddev"]) \
                   + " " + self.tr("cpu_temp_stddev_unit") + " <br>"
        content += self.tr("cpu_temp_max") + ": " + prepare_number_for_representation(data_dict["cpu_temp_max"]) \
                   + " " + self.tr("cpu_temp_max_unit") + " <br>"
        content += self.tr("ram_usage_mean") + ": " + prepare_number_for_representation(data_dict["ram_usage_mean"]) \
                   + " " + self.tr("ram_usage_mean_unit") + " <br>"
        content += self.tr("ram_usage_stddev") + ": " + prepare_number_for_representation(data_dict["ram_usage_stddev"]) \
                   + " " + self.tr("ram_usage_stddev_unit") + " <br>"
        content += self.tr("ram_usage_max") + ": " + prepare_number_for_representation(data_dict["ram_usage_max"]) \
                   + " " + self.tr("ram_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["cpu_usage_core_mean"])):
            content += self.tr("core" + str(i + 1)) + "<br>"
            content += self.tr("cpu_usage_core_mean") + ": " + prepare_number_for_representation(
                data_dict["cpu_usage_core_mean"][i]) \
                       + " " + self.tr("cpu_usage_core_mean_unit") + " <br>"
            content += self.tr("cpu_usage_core_stddev") + ": " + prepare_number_for_representation(
                data_dict["cpu_usage_core_stddev"][i]) \
                       + " " + self.tr("cpu_usage_core_stddev_unit") + " <br>"
            content += self.tr("cpu_usage_core_max") + ": " + prepare_number_for_representation(
                data_dict["cpu_usage_core_max"][i]) \
                       + " " + self.tr("cpu_usage_core_max_unit") + " <br>"
            content += self.tr("cpu_temp_core_mean") + ": " + prepare_number_for_representation(
                data_dict["cpu_temp_core_mean"][i]) \
                       + " " + self.tr("cpu_temp_core_mean_unit") + " <br>"
            content += self.tr("cpu_temp_core_stddev") + ": " + prepare_number_for_representation(
                data_dict["cpu_temp_core_stddev"][i]) \
                       + " " + self.tr("cpu_temp_core_stddev_unit") + " <br>"
            content += self.tr("cpu_temp_core_max") + ": " + prepare_number_for_representation(
                data_dict["cpu_temp_core_max"][i]) \
                       + " " + self.tr("cpu_temp_core_max_unit") + " <br>"

        for i in range(0, len(data_dict["gpu_usage_mean"])):
            content += self.tr("gpu_temp_mean") + ": " + prepare_number_for_representation(
                data_dict["gpu_temp_mean"][i]) \
                       + " " + self.tr("gpu_temp_mean_unit") + " <br>"
            content += self.tr("gpu_temp_stddev") + ": " + prepare_number_for_representation(
                data_dict["gpu_temp_stddev"][i]) \
                       + " " + self.tr("gpu_temp_stddev_unit") + " <br>"
            content += self.tr("gpu_temp_max") + ": " + prepare_number_for_representation(data_dict["gpu_temp_max"][i]) \
                       + " " + self.tr("gpu_temp_max_unit") + " <br>"
            content += self.tr("gpu_usage_mean") + ": " + prepare_number_for_representation(
                data_dict["gpu_usage_mean"][i]) \
                       + " " + self.tr("gpu_usage_mean_unit") + " <br>"
            content += self.tr("gpu_usage_stddev") + ": " + prepare_number_for_representation(
                data_dict["gpu_usage_stddev"][i]) \
                       + " " + self.tr("gpu_usage_stddev_unit") + " <br>"
            content += self.tr("gpu_usage_max") + ": " + prepare_number_for_representation(
                data_dict["gpu_usage_max"][i]) \
                       + " " + self.tr("gpu_usage_max_unit") + " <br>"

        for i in range(0, len(data_dict["interface_name"])):
            content += str(data_dict["interface_name"][i]) + "<br>"
            content += self.tr("message_frequency_mean") + ": " + prepare_number_for_representation(
                data_dict["message_frequency_mean"][i]) \
                       + " " + self.tr("message_frequency_mean_unit") + " <br>"
            content += self.tr("message_frequency_stddev") + ": " + prepare_number_for_representation(
                data_dict["message_frequency_stddev"][i]) \
                       + " " + self.tr("message_frequency_stddev_unit") + " <br>"
            content += self.tr("message_frequency_max") + ": " + prepare_number_for_representation(
                data_dict["message_frequency_max"][i]) \
                       + " " + self.tr("message_frequency_max_unit") + " <br>"
            content += self.tr("bandwidth_mean") + ": " + prepare_number_for_representation(
                data_dict["bandwidth_mean"][i]) \
                       + " " + self.tr("bandwidth_mean_unit") + " <br>"
            content += self.tr("bandwidth_stddev") + ": " + prepare_number_for_representation(
                data_dict["bandwidth_stddev"][i]) \
                       + " " + self.tr("bandwidth_stddev_unit") + " <br>"
            content += self.tr("bandwidth_max") + ": " + prepare_number_for_representation(
                data_dict["bandwidth_max"][i]) \
                       + " " + self.tr("bandwidth_max_unit") + " <br>"

        for i in range(0, len(data_dict["drive_name"])):
            content += data_dict["drive_name"][i] + " <br>"
            content += self.tr("drive_free_space") + ": " + prepare_number_for_representation(
                data_dict["drive_free_space"][i]) \
                       + " " + self.tr("drive_free_space_unit") + " <br>"
            content += self.tr("drive_read") + ": " + prepare_number_for_representation(data_dict["drive_read"][i]) \
                       + " " + self.tr("drive_read_unit") + " <br>"
            content += self.tr("drive_write") + ": " + prepare_number_for_representation(data_dict["drive_write"][i]) \
                       + " " + self.tr("drive_write_unit") + " <br>"
        content += "</p>"

        return content

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["cpu_usage_mean",
                "cpu_usage_stddev", "cpu_usage_max", "cpu_usage_core_mean", "ram_usage_mean",
                "ram_usage_stddev", "ram_usage_max", "cpu_temp_mean", "cpu_temp_stddev", "cpu_temp_max",
                "cpu_usage_core_stddev", "cpu_usage_core_max", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                "gpu_usage_stddev", "gpu_usage_max",
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

        if data_dict["window_stop"] == Time(0):
            return "No data yet"
        elif (Time.now() - data_dict["window_stop"]) > Duration(MAXIMUM_OFFLINE_TIME):
            # last entry was more than MAXIMUM_OFFLINE_TIME ago, it could be offline!
            return "No data since " + prepare_number_for_representation(Time.now() - data_dict["window_stop"]) \
                   + " seconds"

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries_for_log()
        else:
            content += self.tr("cpu_usage_mean") + ": " + prepare_number_for_representation(data_dict["cpu_usage_mean"]) \
                       + " " + self.tr("cpu_usage_mean_unit") + " - "
            content += self.tr("ram_usage_mean") + ": " + prepare_number_for_representation(data_dict["ram_usage_mean"]) \
                       + " " + self.tr("ram_usage_mean_unit") + " - "
            content += self.tr("cpu_temp_mean") + ": " + prepare_number_for_representation(data_dict["cpu_temp_mean"]) \
                       + " " + self.tr("cpu_temp_mean_unit")

        return content

    def get_list_items(self):
        return ["cpu_usage_core_mean", "cpu_usage_core_stddev",
                "cpu_usage_core_max", "cpu_temp_core_mean", "cpu_temp_core_stddev",
                "cpu_temp_core_max", "gpu_temp_mean", "gpu_temp_stddev", "gpu_temp_max", "gpu_usage_mean",
                "gpu_usage_stddev", "gpu_usage_max", "interface_name", "message_frequency_mean",
                "message_frequency_stddev",
                "message_frequency_max", "bandwidth_mean", "bandwidth_stddev", "bandwidth_max",
                "drive_name", "drive_free_space", "drive_read", "drive_write"]

    def get_time_items(self):
        return []
