from rospy.rostime import Time

from python_qt_binding.QtCore import QTranslator

from abstract_item import AbstractItem
from helper_functions import prepare_number_for_representation, UPDATE_FREQUENCY, TOPIC_AGGREGATION_FREQUENCY

from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration
from rospy.rostime import Time


class TopicItem(AbstractItem):
    """
    A TopicItem represents a specific topic which contains many connections and has attributes like the number of sent messages.
    """

    def __init__(self, logger, seuid, first_message, parent=None):
        """Initializes the TopicItem.
        
        :param seuid: the seuid of the item
        :type seuid: str
        :param logger: a logger where to log when special events occur
        :type logger: ModelLogger
        :param parent: the parent-item
        :type parent: AbstractItem
        """
        AbstractItem.__init__(self, logger, seuid, parent)
        self.__parent = parent
        self._type = "topic"

        self._attributes = []
        # todo: currently probably only these 4 implemented
        self._attributes.extend(["dropped_msgs", "traffic",
                                 "stamp_age_mean", "stamp_age_max", "stamp_age_stddev", "period_max"])


        if hasattr(first_message, "delivered_msgs"):
            self._attributes.append("delivered_msgs")

        for item in self._attributes:
            self._add_data_list(item)

        #self._attributes.remove("traffic")
        self._attributes.append("bandwidth")

        self.__rated_attributes = []

        self._attributes.append("packages")
        self._attributes.append("packages_per_second")
        self._attributes.append("frequency")

        self.__calculated_data = {}
        for key in self._attributes:
            self.__calculated_data[key] = []

        self.__calculated_data["window_start"] = []
        self.__calculated_data["window_stop"] = []

        for item in self._attributes:
            self.__rated_attributes.append(item + ".actual_value")
            self.__rated_attributes.append(item + ".expected_value")
            self.__rated_attributes.append(item + ".state")

        for item in self.__rated_attributes:
            self._add_rated_data_list(item)

        self._logger.log("info", Time.now(), seuid, "Created a new TopicItem")

        self.__timer = Timer(Duration(nsecs=TOPIC_AGGREGATION_FREQUENCY), self.__aggregate_topic_data)

    def get_items_younger_than(self, time, *args):
        """
        Used to overwrite the standart implementation in AbstractItem. This method provides the data from the
        calculated data and *not* from the raw input. This is especially wanted when plotting
        :param time:
        :param args:
        :return:
        """
        self._data_lock.acquire()
        return_values = {}

        if args:
            for key in args:
                return_values[key] = None
            if "window_stop" not in args:
                return_values["window_stop"] = None
        else:
            for key in self.__calculated_data:
                return_values[key] = None

        breakpoint = 0
        list_of_time = self.__calculated_data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    try:
                        return_values[key] = self.__calculated_data[key][:]
                    except KeyError:
                        print("Accessed key was: " + key + ". Available keys are: ")
                        print(self.__calculated_data)
                        raise
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self.__calculated_data:
                                return_values[key] = self.__calculated_data[key][breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i,
                                                 len(self.__calculated_data[key]), key)
                        break

        self._data_lock.release()
        return return_values


    def get_raw_items_younger_than(self, time, *args):
        """
        Returns all entries that are younger than time either in all keys of self._data or if args not empty in
        all key corresponding to args.
        Warning: Method assumes data is sorted by time if this is not true will return too few or too much data.

        :param time: the lower bound in seconds
        :type time: rospy.Time
        :param args: the keys to the dict
        :type args: str

        :returns: dict of lists
        :rtype: dict
        :raises KeyError: if an element in args cannot be found in any of the dictionaries (data vs rated data)
        """
        return_values = {}

        if args:
            for key in args:
                return_values[key] = None
            if "window_stop" not in args:
                return_values["window_stop"] = None
        else:
            for key in self._data:
                return_values[key] = None

        breakpoint = 0
        list_of_time = self._data["window_stop"]
        length = len(list_of_time)

        if length is not 0:
            if list_of_time[0] >= time:
                for key in return_values:
                    try:
                        return_values[key] = self._data[key][:]
                    except KeyError:
                        print("Accessed key was: " + key + ". Available keys are: ")
                        print(self._data)
                        raise
            else:
                for i in range(length - 1, -1, -1):
                    if list_of_time[i] < time:
                        breakpoint = i + 1
                        for key in return_values:
                            if key in self._data:
                                return_values[key] = self._data[key][breakpoint:length]
                            else:
                                raise IndexError("IndexError! length of the list %s, accessed index %s. length of data"
                                                 " at given point %s, key is %s", length, i, len(self._data[key]), key)
                        break

        return return_values


    def __aggregate_topic_data(self, event):
        """
        Aggregates the topic every TOPIC_AGGREGATION_FREQUENCY nsecs and pushes the updated data to
        self.__calculated_data.

        :param event: containing information when this method was called - not used but needed for the interface
        """
        self._data_lock.acquire()
        entries = self.get_raw_items_younger_than(Time.now() - Duration(secs=TOPIC_AGGREGATION_FREQUENCY))
        # print(entries)
        #print(len(entries["window_stop"]))
        #print(entries["window_start"])

        #print(self.__calculated_data)
        for key in self.__calculated_data.keys():
            self.__calculated_data[key].append(0)

        #print(self.__calculated_data)
        self.__calculated_data["window_start"][-1] = Time.now()
        self.__calculated_data["window_stop"][-1] = Time.now() - Duration(secs=600)

        for i in range(0, len(entries["window_stop"])):
            #print("aggregating entry")
            window_len = entries["window_stop"][i] - entries["window_start"][i]
            #scale = 1 / window_len.to_sec()
            self.__calculated_data["window_start"][-1] = min(entries["window_start"][i],
                                                             self.__calculated_data["window_start"][-1])
            self.__calculated_data["window_stop"][-1] = max(entries["window_stop"][i],
                                                            self.__calculated_data["window_stop"][-1])

            if "delivered_msgs" in self.__calculated_data:
                self.__calculated_data["delivered_msgs"][-1] += entries["delivered_msgs"][i]
                self.__calculated_data["frequency"][-1] += entries["delivered_msgs"][i] / window_len.to_sec()
            self.__calculated_data["dropped_msgs"][-1] += entries["dropped_msgs"][i]
            self.__calculated_data["traffic"][-1] += entries["traffic"][i]
            self.__calculated_data["stamp_age_max"][-1] = max(entries["stamp_age_max"][i].to_sec(),
                                                              self.__calculated_data["stamp_age_max"][-1])
            self.__calculated_data["period_max"][-1] = max(entries["period_max"][i].to_sec(),
                                                           self.__calculated_data["period_max"][-1])
            self.__calculated_data["packages"][-1] += 1
            #print("in loop")
            #print(self.__calculated_data)

        window_len = self.__calculated_data["window_stop"][-1] - self.__calculated_data["window_start"][-1]
        #print(self.__calculated_data["window_stop"][-1])
        if window_len is not 0:
            self.__calculated_data["bandwidth"][-1] = self.__calculated_data["traffic"][-1] / window_len.to_sec()
            self.__calculated_data["packages_per_second"][-1] = self.__calculated_data["packages"][-1] / window_len.to_sec()

        self._data_lock.release()
        #print("PRINT")
        #for key in self.__calculated_data:
        #    print(key)
        #    print(self.__calculated_data[key][-1])


    def execute_action(self, action):
        """
        Not senseful, Topics cannot execute actions.

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
        for key in self.__calculated_data:
            if self.__calculated_data[key]:
                #print(self.__calculated_data[key][-1])
                data_dict[key] = self.__calculated_data[key][-1]
            else:
                data_dict[key] = self.tr("Currently no value available")

        data_dict["state"] = self.get_state()

        content = "<p class=\"detailed_data\">"

        content += self.get_erroneous_entries()

        content += "Rounded to a second:<br>"

        if "delivered_msgs" in self._attributes:
            content += self.tr("delivered_msgs") + ": " + prepare_number_for_representation(data_dict["delivered_msgs"]) \
                   + " " + self.tr("delivered_msgs_unit") + " <br>"
            content += self.tr("frequency") + ": " + prepare_number_for_representation(data_dict["frequency"]) \
                   + " " + self.tr("frequency_unit") + " <br>"


        content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(data_dict["dropped_msgs"]) \
                   + " " + self.tr("dropped_msgs_unit") + " <br>"
        content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) \
                   + " " + self.tr("traffic_unit") + " <br>"
        content += self.tr("period_max") + ": " + prepare_number_for_representation(data_dict["period_max"]) \
                   + " " + self.tr("period_max_unit") + " <br>"
        content += self.tr("stamp_age_max") + ": " + prepare_number_for_representation(data_dict["stamp_age_max"]) \
                   + " " + self.tr("stamp_age_max_unit") + " <br>"
        content += self.tr("bandwidth") + ": " + prepare_number_for_representation(data_dict["bandwidth"]) \
                   + " " + self.tr("bandwidth_unit") + " <br>"
        content += self.tr("packages_per_second") + ": " + prepare_number_for_representation(
            data_dict["packages_per_second"]) \
                   + " " + self.tr("packages_per_second_unit") + " <br>"

        content += "</p>"
        return content


    def get_plotable_items(self):
        """
        Returns items for the plot.

        :returns: str[]
        """
        if "delivered_msgs" in self.__calculated_data:
            return ["dropped_msgs", "traffic", "stamp_age_max", "period_max", "packages",
                    "bandwidth", "delivered_msgs", "frequency"]

        else:
            return ["dropped_msgs", "traffic", "stamp_age_max", "period_max", "packages", "bandwidth"]


    def get_short_data(self):
        """
        Returns a shortend version of the item data.

        :returns: data of the item
        :rtype: str
        """
        data_dict = {}
        for key in self.__calculated_data:
            if self.__calculated_data[key]:
                data_dict[key] = self.__calculated_data[key][-1]
            else:
                data_dict[key] = self.tr("Currently no value available")

        data_dict["state"] = self.get_state()

        content = ""
        if data_dict["state"] is "error":
            content += self.get_erroneous_entries_for_log()
        else:
            content += self.tr("dropped_msgs") + ": " + prepare_number_for_representation(
                data_dict["dropped_msgs"]) + " " \
                       + self.tr("dropped_msgs_unit") + " - "
            content += self.tr("traffic") + ": " + prepare_number_for_representation(data_dict["traffic"]) + " " \
                       + self.tr("traffic_unit") + " - "
            content += self.tr("stamp_age_mean") + ": " + prepare_number_for_representation(data_dict["stamp_age_mean"]) \
                       + " " + self.tr("stamp_age_mean_unit")

        return content


    def can_execute_actions(self):
        """
        This item cannot execute actions, so it returns False

        :return: False
        """
        return False


    def get_list_items(self):
        return []


    def get_time_items(self):
        return ["stamp_age_mean", "stamp_age_max"]
