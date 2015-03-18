from python_qt_binding.QtCore import QTranslator

from rospy import Time, Duration

import genpy

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation


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

        self.add_keys=["dropped_msgs", "traffic", "delivered_msgs"]
        self.avg_keys=["period_mean", "period_stddev", "stamp_age_mean", "stamp_age_stddev", "bandwidth", "frequency"]
        self.max_keys=["period_max", "stamp_age_max"]

        self._attributes = []
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max", "bandwidth", "frequency"])

        if hasattr(first_message, "delivered_msgs"):
            self._attributes.append("delivered_msgs")
        for item in self._attributes:
            self._add_data_list(item)

        #for element in ["period_mean", "period_stddev", "period_max"]:
        #    self._attributes.remove(element)

        self.__rated_attributes = []
        self.__rated_attributes.append("alive.actual_value")
        self.__rated_attributes.append("alive.expected_value")
        self.__rated_attributes.append("alive.state")
        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")


        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new ConnectionItem")

    def append_data(self, message):
        """
        Appends data to the data of the AbstractItem.

        :param message: the message to append
        :type message: one of the different message types TopicStatistics, HostStatistics or NodeStatistics
        :raises KeyError: if an entry is in the rated dictionary but not found in the message
        """
        self._data_lock.acquire()
        for attribute in self._data:
            if attribute is "frequency":
                self._data[attribute].append(message.delivered_msgs / (message.window_stop - message.window_start).to_sec())
            elif attribute is "bandwidth":
                self._data[attribute].append(message.traffic / (message.window_stop - message.window_start).to_sec())
            else:
                self._data[attribute].append(getattr(message, attribute))

        #self.__state.append("unknown")
        self._length_of_data += 1
        #self._update_current_state()
        self._data_lock.release()


    def aggregate_data(self, period):
        """
        :param period: The amount in seconds over which the data should be aggregated.
        :return:
        """

        values = {}
        for key in self._attributes:
            values[key] = 0

        entries = self.get_items_younger_than(Time.now() - (Duration(secs=period) if int(Duration(secs=period).to_sec()) <= int(Time.now().to_sec()) else Time(0) ))


        length = len(entries["window_stop"])

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

        if type(data_dict["window_stop"]) != unicode and type(data_dict["window_start"]) != unicode:
            window_len = data_dict["window_stop"] - data_dict["window_start"]
        else:
            window_len = 0
        if "delivered_msgs" in self._attributes:
            content += self.tr("delivered_msgs") + ": " + prepare_number_for_representation(data_dict["delivered_msgs"]) \
                       + " " + self.tr("delivered_msgs_unit") + " <br>"
            if window_len is not 0:
                content += self.tr("frequency") + ": " + prepare_number_for_representation(data_dict["delivered_msgs"]
                                                                                           / window_len.to_sec()) \
                           + " " + self.tr("frequency_unit") + " <br>"

        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) + " " \
                   + self.tr("dropped_msgs_unit") + " <br>"
        #content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
        #           + self.tr("traffic_unit") + " <br>"
        if window_len is not 0:
            content += self.tr("bandwidth") + ": " + prepare_number_for_representation(data_dict["traffic"] /
                                                                                       window_len.to_sec()) \
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
        return ["dropped_msgs", "traffic", "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]


    def get_short_data(self):
        """
        Returns a shortend version of the item data.
        
        :returns: data of the item
        :rtype: str
        """
        data_dict = self.get_latest_data()
        if Time.now() - data_dict["window_stop"] > Duration(secs=5):
          return "No recent data"

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries_for_log()
            pass
        else:
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(
                data_dict["dropped_msgs"]) + " " \
                       + self.tr("dropped_msgs_unit") + " - "
            #content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
            #           + self.tr("traffic_unit") + " - "
            content += self.tr("period_mean") + ": " + prepare_number_for_representation(data_dict["period_mean"]) \
                       + " " + self.tr("period_mean_unit") + "  - "
            content += self.tr("stamp_age_mean") + ": " + prepare_number_for_representation(data_dict["stamp_age_mean"]) \
                       + " " + self.tr("stamp_age_mean_unit")

        return content

    def get_list_items(self):
        return []

    def get_time_items(self):
        return ["period_mean", "period_stddev", "period_max", "stamp_age_mean",
                "stamp_age_stddev", "stamp_age_max"]
