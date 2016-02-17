from rospy.rostime import Time

from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation


class RootItem(AbstractItem):
    """
    The RootItem represents the parent of the ROSModel
    """
    
    def __init__(self, logger, seuid, parent=None, *args):
        """
        Initializes the TItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: QObject
        :param args:
        :type args: str
        """
        AbstractItem.__init__(self, logger, "root", parent)
        self._attributes = []
        self._attributes.extend(["cpu_usage_mean", "cpu_temp_mean", "cpu_usage_max", "cpu_temp_max",
                                        "ram_usage_mean", "ram_usage_max", "total_traffic", "connected_hosts",
                                        "connected_nodes", "topic_counter", "connection_counter"])
        self._type = "root"
        
        for item in self._attributes:
            self._add_data_list(item)

        self.__rated_attributes = []
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

    def _updateTimer(self, event):
        self.alive = True

    def append_data_dict(self, data):
        """
        Appends data to the data of the AbstractItem.

        :param data: the data to append in key value form
        :type data: dict
        :raises KeyError: if an entry is in the global data dictionary but not found in the given dictionary
        """
        self._data_lock.acquire()
        self.alive = True
        if "window_stop" not in data:
            data["window_stop"] = Time.now()

        for attribute in self._data:
            if attribute in data:
                self._data[attribute].append(data[attribute])
            else:
                self._data[attribute].append(None)

        if "state" in data:
            self.add_state((data["state"]))
        else:
            self.add_state("unknown")

        self._length_of_data += 1
        self._update_current_state()
        self._data_lock.release()

    def get_detailed_data(self):
        """
        Returns the detailed data of the RootItem.
        
        :returns: detailed data
        :rtype: str
        """
        data_dict = self.get_latest_data()

        content = "<p class=\"detailed_data_overview\">"

        content += self.tr("total_traffic") + ": " + prepare_number_for_representation(data_dict["total_traffic"]) + " " + self.tr("bandwidth_mean_unit") + " <br>"
        content += self.tr("connected_hosts") + ": " + str(data_dict["connected_hosts"]) + "<br>"
        content += self.tr("connected_nodes") + ": " + str(data_dict["connected_nodes"]) + "<br>"
        content += self.tr("topic_counter") + ": " + str(data_dict["topic_counter"]) + "<br>"
        content += self.tr("connection_counter") + ": " + str(data_dict["connection_counter"]) + "<br>"
        content += self.tr("cpu_usage_max") + ": " + prepare_number_for_representation(data_dict["cpu_usage_max"]) + " " + self.tr("cpu_usage_max_unit") + " <br>"
        content += self.tr("cpu_temp_mean") + ": " + ("unknown" if prepare_number_for_representation(data_dict["cpu_temp_mean"]) is 0
                                        else prepare_number_for_representation(data_dict["cpu_temp_mean"])) + " " + self.tr("cpu_temp_mean_unit") + " <br>"
        content += self.tr("ram_usage_mean") + ": " + prepare_number_for_representation(data_dict["ram_usage_mean"]) + " " + self.tr("ram_usage_mean_unit") + " <br>"
        content += self.tr("cpu_usage_mean") + ": " + prepare_number_for_representation(data_dict["cpu_usage_mean"]) + " " + self.tr("cpu_usage_mean_unit") + " <br>"
        content += self.tr("cpu_temp_max") + ": " + ("unknown" if prepare_number_for_representation(data_dict["cpu_temp_max"]) is 0
                                       else prepare_number_for_representation(data_dict["cpu_temp_max"])) + " " + self.tr("cpu_temp_max_unit") + " <br>"
        content += self.tr("ram_usage_max") + ": " + prepare_number_for_representation(data_dict["ram_usage_max"]) + " " + self.tr("ram_usage_max_unit") + " <br>"

        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["cpu_usage_mean", "cpu_temp_mean", "cpu_usage_max", "cpu_temp_max",
                "ram_usage_mean", "ram_usage_max", "total_traffic", "connected_hosts",
                "connected_nodes", "topic_counter", "connection_counter"]

    def units_of_plotable_items(self):
        """
        Returns the units of the items for the plot.
        
        :returns str[]
        """
        return {"average_ram_load": "%",
                "cpu_usage_mean": "%",
                "total_traffic": "%"
                }

    def get_short_data(self):
        return NotImplementedError()

    def can_execute_actions(self):
        """
        This item cannot execute actions, so it returns False

        :return: False
        """
        return False