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

        self.add_keys=["dropped_msgs", "traffic"]
        self.avg_keys=["period_mean", "period_stddev", "stamp_age_mean", "stamp_age_stddev", "bandwidth", "frequency"]
        self.max_keys=["period_max", "stamp_age_max"]

        self._attributes = []
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "period_mean", "period_stddev", "period_max", "stamp_age_mean",
                                 "stamp_age_stddev", "stamp_age_max", "bandwidth", "frequency"])

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

    def get_latest_data(self, parent, *args):
        """
        Returns the latest dict of the data_list or the item of the dict with the given key.
        IMPORTANT: This was reimplemented here because the code needs to be adapted to the current parent in
        the QT Model.

        :param kwargs: the keys to the dict
        :type kwargs: str
        :type parent: TopicItem

        :returns: dict of the item
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data) or attributes (namely name, type, data and state)
        """
        self._data_lock.acquire()
        return_dict = {}

        if parent.is_subscriber(ROSModel().parent())

        if args:
            for key in args:
                if key is 'name':
                    return_dict['name'] = self.seuid
                elif key is 'type':
                    return_dict['type'] = self._type
                    # elif key is 'data':
                    # return_dict['data'] = self.get_short_data()
                elif key is 'state':
                    if self.__state:
                        return_dict['state'] = self.get_state()
                    else:
                        return_dict["state"] = "unknown"
                else:
                    if key in self._data:
                        if self._data[key]:
                            return_dict[key] = self._data[key][-1]
                        else:
                            if key == 'window_stop':
                                return_dict[key] = Time(0)
                            elif key in self.get_list_items():
                                return_dict[key] = [self.tr("Currently no value available")]
                            else:
                                return_dict[key] = self.tr("Currently no value available")
                    elif key in self._rated_data:
                        if self._rated_data[key]:
                            return_dict[key] = self._rated_data[key][-1]
                    else:
                        raise KeyError("item " + key + "was not found")
        else:
            return_dict['name'] = self.seuid
            return_dict['type'] = self._type
            # return_dict['data'] = self.get_short_data()
            for entry in self._data:
                if self._data[entry]:
                    return_dict[entry] = self._data[entry][-1]
                else:
                    if entry == 'window_stop':
                        return_dict[entry] = Time(0)
                    elif entry in self.get_list_items():
                        return_dict[entry] = [self.tr("Currently no value available")]
                    else:
                        return_dict[entry] = self.tr("Currently no value available")
            for entry in self._rated_data:
                if entry == 'window_start' or entry == 'window_stop':
                    continue
                if self._rated_data[entry]:
                    return_dict[entry] = self._rated_data[entry][-1]
                else:
                    return_dict[entry] = self.tr("Currently no value available")
            if self.__state:
                return_dict['state'] = self.get_state()
            else:
                return_dict['state'] = "unknown"

        self._data_lock.release()
        return return_dict



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
        #print(data_dict["window_stop"])
        #print(type(data_dict["window_stop"]))
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
