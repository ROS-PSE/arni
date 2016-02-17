from python_qt_binding.QtCore import QTranslator

from rospy.rostime import Time, Duration

import genpy

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation, MAXIMUM_OFFLINE_TIME, ROUND_DIGITS


class ConnectionItem(AbstractItem):
    """
    A ConnectionItem reresents the connection between a publisher and a subscriber and the topic they are publishing / listening on
    """

    def __init__(self, logger, seuid, first_message, parent=None):
        """
        Initializes the ConnectionItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param type: the type of the item
        :type type: str
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        self.__parent = parent
        self._type = "connection"

        self.add_keys=["dropped_msgs", "traffic"]
        self.avg_keys=["period_mean", "period_stddev", "stamp_age_mean", "stamp_age_stddev", "bandwidth", "frequency"]
        self.max_keys=["period_max", "stamp_age_max"]

        self._attributes = []
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max", "bandwidth", "frequency"])

        for item in self._attributes:
            self._add_data_list(item)

        for item in self._attributes:
            self._rated_attributes.append(item + ".actual_value")
            self._rated_attributes.append(item + ".expected_value")
            self._rated_attributes.append(item + ".state")


        for item in self._rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new ConnectionItem")

        self.show_as_subscriber = False
        self.tree_item1 = None
        self.tree_item2 = None


    def aggregate_data(self, period):
        """
        :param period: The amount in seconds over which the data should be aggregated.
        :return:
        """

        values = {}
        for key in self._attributes:
            values[key] = 0

        entries = self.get_items_younger_than(Time.now() - (Duration(secs=period) if int(Duration(secs=period).to_sec()) <= int(Time.now().to_sec()) else Time(0) ))


        length = len(entries["window_stop"]) if entries["window_stop"] else 0

        if length > 0:
            for key in self.add_keys:
                for i in range(0, length):
                    values[key] += entries[key][i]
            for key in self.max_keys:
                if type(entries[key][-1]) == genpy.rostime.Time or type(entries[key][-1]) == genpy.rostime.Duration:
                    for i in range(0, length):
                        if entries[key][i].to_sec() > values[key]:
                            values[key] = entries[key][i].to_sec()
                else:
                    for i in range(0, length):
                        if entries[key][i] > values[key]:
                            values[key] = entries[key][i]
            for key in self.avg_keys:
                if type(entries[key][0]) is genpy.rostime.Time or type(entries[key][0]) is genpy.rostime.Duration:
                    for i in range(0, length):
                        values[key] += entries[key][i].to_sec()
                else:
                    for i in range(0, length):
                        values[key] += entries[key][i]
                values[key] = values[key] / length

        return values




    def execute_action(self, action):
        """
        Not senseful, Connection cannot execute actions.

        :param action: action to be executed
        :type action: RemoteAction
        """
        pass

    def get_detailed_data(self):
        """
        Returns the detailed data of the ConnectionItem.
        
        :returns: str
        """
        data_dict = self.get_latest_data()
        if Time.now() - data_dict["window_stop"] > Duration(secs=5):
          return "No recent data"

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        if "frequency" in self._attributes:
            content += self.tr("frequency") + ": " + prepare_number_for_representation(data_dict["frequency"]) \
                       + " " + self.tr("frequency_unit") + " <br>"

        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) + " " \
                   + self.tr("dropped_msgs_unit") + " <br>"
        content += self.tr("bandwidth") + ": " + prepare_number_for_representation(data_dict["bandwidth"]) + " " \
                    + " " + self.tr("bandwidth_unit") + " <br>"
        content += self.tr("period_mean") + ": " + prepare_number_for_representation(data_dict["period_mean"]) \
                   + " " + self.tr("period_mean_unit") + " <br>"
        content += self.tr("period_stddev") + ": " + prepare_number_for_representation(data_dict["period_stddev"]) \
                   + " " + self.tr("period_stddev_unit") + " <br>"
        content += self.tr("period_max") + ": " + prepare_number_for_representation(data_dict["period_max"]) + " " \
                   + self.tr("period_max_unit") + " <br>"
        content += self.tr("stamp_age_mean") + ": " + prepare_number_for_representation(data_dict["stamp_age_mean"]) \
                   + " " + self.tr("stamp_age_mean_unit") + " <br>"
        content += self.tr("stamp_age_stddev") + ": " + prepare_number_for_representation(data_dict["stamp_age_stddev"]) \
                   + " " + self.tr("stamp_age_stddev_unit") + " <br>"
        content += self.tr("stamp_age_max") + ": " + prepare_number_for_representation(data_dict["stamp_age_max"]) \
                   + " " + self.tr("stamp_age_max_unit") + " <br>"
        content += "</p>"

        return content

    def get_plotable_items(self):
        """
        Returns items for the plot.
        
        :returns: str[]
        """
        return ["dropped_msgs", "bandwidth", "frequency", "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]


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
            content += self.tr("frequency") + ": " + prepare_number_for_representation(data_dict["frequency"]) \
                       + " " + self.tr("frequency_unit") + "  - "
            content += self.tr("bandwidth") + ": " + prepare_number_for_representation(
                data_dict["bandwidth"]) + " " \
                       + self.tr("bandwidth_unit") + " - "
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) \
                       + " " + self.tr("dropped_msgs_unit")

        return content

    def get_list_items(self):
        return []

    def get_time_items(self):
        return ["period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]
